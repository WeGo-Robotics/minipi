#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, requests, sys, re, time, threading
from requests.adapters import HTTPAdapter
import rospy
from std_msgs.msg import String
from yolo11_detect_pkg.msg import Yolo
from sim2real_msg.msg import ControlState
from geometry_msgs.msg import Twist
import numpy as np
from math import pi

RK_BASE = os.environ.get("RK_BASE", "http://127.0.0.1:8080")
MODEL   = os.environ.get("RK_MODEL", "Qwen2.5-3B")

class RkllamaPi:
    def __init__(self):
        # ── 경량 System Prompt (라벨/지시문 금지 규칙 포함) ───────────────────────
        self.BASE_SYS = (
            "You are a robot tool router. If user asks 'what can you see' -> call check_around. "
            "If 'go to X' -> call go_to_object with X. Otherwise answer with one short friendly sentence. "
            "Output must be only Korean or English. "
            "Do NOT include any labels or meta text: no brackets [], no parentheses (), no hashtags #, "
            "no prefixes like [LLM] or [User], and no stage directions."
        )
        self.TOOLS = [
            {"type": "function", "function": {"name": "check_around", "description": "list visible objects"}},
            {"type": "function", "function": {
                "name": "go_to_object",
                "description": "go to a named object",
                "parameters": {"type": "object","properties":{"object_name":{"type":"string"}},"required":["object_name"]}
            }},
        ]

        # ── HTTP 세션 재사용 (keep-alive) ───────────────────────────────────────
        self.http = requests.Session()
        self.http.mount("http://", HTTPAdapter(pool_connections=4, pool_maxsize=8, max_retries=0))
        self.http.headers.update({"Connection": "keep-alive"})

        # ── ROS 초기화 ────────────────────────────────────────────────────────
        rospy.init_node("rkllama_pi")
        self.usr_input   = rospy.Subscriber("/gui/input", String, self.usr_input_cb)
        self.yolo_input  = rospy.Subscriber("/yolo/detections", Yolo, self.yolo_input_cb)

        self.rkllama_output = rospy.Publisher("/rkllama/output", String, queue_size=10)
        self.cmd_vel_pub    = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.joy_dance_pub  = rospy.Publisher("/joy_msg", ControlState, queue_size=10)

        self.joy_msg = ControlState()
        self.last_yolo_data = None

        # 이동 상태
        self.is_moving = False
        self.target_object_name = None
        self.is_aligned = False
        self.alignment_time = 0.0

        # 춤 상태
        self.is_dancing = False
        self.dance_end_time = 0.0

        # 10Hz 주기 제어
        rospy.Timer(rospy.Duration(0.1), self.move_to_target_cb)

    # ─────────────────────────── 공통 유틸 ────────────────────────────────
    def sanitize_text(self, text: str) -> str:
        """모델 출력/로컬 메시지에서 라벨·해시·괄호 메모 제거"""
        if not text:
            return ""
        s = text.strip()
        # 선행 라벨 [LLM] [User] 제거
        s = re.sub(r'^\s*\[[^\]]+\]\s*', '', s)
        # 줄 끝 해시 주석 비슷한 것 제거
        s = re.sub(r'\s#[^#\n]+$', '', s)
        s = re.sub(r'\s+#\s*$', '', s)
        # 괄호 속 메모 제거 (단순 전략)
        s = re.sub(r'\([^)]*\)', '', s)
        # 잔여 대괄호 메모 방어적 제거
        s = re.sub(r'\[[^\]]*\]', '', s)
        # 중복 공백 정리
        s = re.sub(r'\s{2,}', ' ', s).strip()
        return s

    def publish_clean(self, text: str):
        self.rkllama_output.publish(self.sanitize_text(text))

    # ─────────────────────────── 콜백/핸들러 ─────────────────────────────
    def yolo_input_cb(self, msg: Yolo):
        self.last_yolo_data = msg

    def usr_input_cb(self, msg: String):
        user_input = (msg.data or "").strip()
        threading.Thread(target=self._handle_input_worker, args=(user_input,), daemon=True).start()

    def _handle_input_worker(self, user_input: str):
        rospy.loginfo(f"User Input: {user_input}")

        # ── Fast Path: 아주 간단한 인사/명령은 LLM 우회 ─────────────────────────
        if user_input.lower() in ["hi", "hello", "안녕", "ㅎㅇ"]:
            self.publish_clean("안녕하세요! 무엇을 도와드릴까요?")
            return

        m = re.match(r'^\s*go\s*to\s+(.+)$', user_input, re.I)
        if m:
            obj = m.group(1).strip()
            tool_result = self.execute_go_to_object({"object_name": obj})
            self.publish_clean(tool_result.get("message", ""))
            return

        if re.search(r'(what.*see|주변.*(보이|있)|뭐가\s*있|보이는\s*게)', user_input, re.I):
            tool_result = self.execute_check_around()
            self.publish_clean(tool_result.get("message", ""))
            return

        # ── LLM 한 번만 호출 ────────────────────────────────────────────────
        messages = [
            {"role": "system", "content": self.BASE_SYS},
            {"role": "user", "content": user_input}
        ]
        try:
            resp = self.chat_once(messages, self.TOOLS)

            # 도구 호출이면 실행 후 로컬 한 줄 응답
            if resp.get("tool_calls"):
                tc = resp["tool_calls"][0]
                fname = tc["function"]["name"]
                try:
                    fargs = json.loads(tc["function"].get("arguments", "{}"))
                except json.JSONDecodeError:
                    fargs = {}

                if fname == "go_to_object" and not fargs.get("object_name"):
                    mm = re.search(r'go\s*to\s+(.+)', user_input, re.I)
                    if mm:
                        fargs["object_name"] = mm.group(1).strip()

                if fname == "check_around":
                    tool_result = self.execute_check_around()
                elif fname == "go_to_object":
                    tool_result = self.execute_go_to_object(fargs)
                else:
                    tool_result = {"status": "error", "message": f"Unknown tool {fname}"}

                self.publish_clean(tool_result.get("message", ""))
                return

            # 일반 답변
            final_text = (resp.get('message', {}) or {}).get('content') or resp.get('content') or ""
            final_text = final_text.strip() or "요청을 이해하지 못했습니다."
            self.publish_clean(final_text)

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"API error: {e}")
            self.publish_clean(f"LLM 서버 호출 오류: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            self.publish_clean(f"처리 중 예기치 않은 오류 발생: {e}")

    # ─────────────────────────── 도구 구현 ───────────────────────────────
    def execute_check_around(self):
        rospy.loginfo("[CHECK_AROUND] invoked.")
        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", []):
            return {"status": "success", "objects_found": [], "message": "주변에 아무것도 보이지 않아요."}

        object_list = [(d.label, d.x_center, d.y_center) for d in self.last_yolo_data.detections]
        names = sorted(list(set([o[0] for o in object_list])))

        if not names:
            msg = "주변에 아무것도 보이지 않아요."
        elif len(names) == 1:
            msg = f"주변에 {names[0]} 하나가 보여요."
        elif len(names) == 2:
            msg = f"주변에 {names[0]}랑 {names[1]}가 보여요."
        else:
            msg = f"주변에 {', '.join(names[:-1])}, 그리고 {names[-1]}가 보여요."

        rospy.loginfo(f"[CHECK_AROUND] objects: {names}")
        return {"status": "success", "objects_found": names, "message": msg}

    def execute_go_to_object(self, args):
        object_name = (args or {}).get("object_name")
        rospy.loginfo(f"[GO_TO] requested: object_name={object_name}")

        if not object_name:
            return {"status": "error", "message": "이동할 물체의 이름이 지정되지 않았습니다."}
        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", []):
            return {"status": "error", "message": f"현재 화면에서 '{object_name}'을(를) 찾을 수 없어 이동을 시작할 수 없습니다."}

        self.target_object_name = object_name
        self.is_moving = True
        self.is_aligned = False
        self.cmd_vel_pub.publish(Twist())  # 잔여 명령 정지

        rospy.loginfo(f"[GO_TO] start moving to {object_name}. Timer will control.")
        return {"status": "success", "message": f"'{object_name}'(으)로 이동을 시작합니다."}

    # ─────────────────────────── 춤 동작 ─────────────────────────────────
    def start_dance_sequence(self):
        if self.is_moving:
            return
        DANCE_DURATION = 3.0
        self.is_dancing = True
        self.dance_end_time = rospy.get_time() + DANCE_DURATION

        rospy.loginfo(f"[DANCE] start for {DANCE_DURATION} sec.")
        self.joy_msg.running_standby_switch = 1.0
        self.joy_dance_pub.publish(self.joy_msg)
        rospy.sleep(0.05)

        self.joy_msg.running_standby_switch = 0.0
        self.joy_msg.candidate_left = 1.0
        self.joy_msg.candidate_right = -1.0
        self.joy_msg.quit = 1.0
        self.joy_msg.knee_angle_increase = 1.0
        self.joy_dance_pub.publish(self.joy_msg)

    # ─────────────────────────── 주기 제어 콜백 ──────────────────────────
    def move_to_target_cb(self, event):
        if self.is_dancing:
            now = rospy.get_time()
            if now < self.dance_end_time:
                rospy.loginfo(f"[DANCE] waiting {self.dance_end_time - now:.2f}s")
                return
            rospy.loginfo("[DANCE] finished.")
            self.cmd_vel_pub.publish(Twist())
            self.is_dancing = False
            self.is_moving = False
            self.target_object_name = None
            self.publish_clean("춤 동작을 마쳤습니다.")

            self.joy_msg.running_standby_switch = 1.0
            self.joy_msg.candidate_left = 0.0
            self.joy_msg.candidate_right = 0.0
            self.joy_msg.quit = 0.0
            self.joy_msg.knee_angle_increase = 0.0
            self.joy_dance_pub.publish(self.joy_msg)
            return

        if not self.is_moving or self.target_object_name is None:
            return

        object_name = self.target_object_name

        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", []):
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(f"[MOVE] no YOLO data. stop '{object_name}'.")
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            return

        target_detection = None
        for d in self.last_yolo_data.detections:
            if (d.label or "").lower() == object_name.lower():
                target_detection = d
                break

        if target_detection is None:
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(f"[MOVE] lost target '{object_name}'. stop.")
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            return

        # 제어 파라미터
        LINEAR_SPEED = 0.35
        MIN_LINEAR_SPEED = 0.2
        PIXEL_TOLERANCE = 50
        FINAL_STOP_Y_CENTER = 400
        ALIGNMENT_HOLD_TIME = 0.8

        cx = target_detection.x_center
        image_width = getattr(self.last_yolo_data, 'image_width', 640)
        center_x = image_width / 2.0
        error_x = center_x - cx

        twist = Twist()
        calculated_angular_z = error_x * (pi / image_width)
        should_stop = False

        if target_detection.y_center >= FINAL_STOP_Y_CENTER:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            should_stop = True
            self.is_aligned = False
            rospy.loginfo("[MOVE] reached near target. stop.")
        elif self.is_aligned:
            now = rospy.get_time()
            if now - self.alignment_time < ALIGNMENT_HOLD_TIME:
                twist.linear.x = LINEAR_SPEED
                twist.angular.z = 0.0
                rospy.loginfo(f"[MOVE] hold straight ({now - self.alignment_time:.2f}s).")
            else:
                self.is_aligned = False
                rospy.loginfo("[MOVE] hold done. back to normal.")
        elif abs(error_x) < PIXEL_TOLERANCE:
            self.is_aligned = True
            self.alignment_time = rospy.get_time()
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = 0.0
            rospy.loginfo("[MOVE] aligned -> go straight & hold.")
        else:
            twist.linear.x = MIN_LINEAR_SPEED
            twist.angular.z = float(np.clip(calculated_angular_z, -0.6, 0.6))
            rospy.loginfo(f"[MOVE] aligning L:{twist.linear.x:.2f} A:{twist.angular.z:.2f}")

        self.cmd_vel_pub.publish(twist)

        if should_stop:
            rospy.loginfo(f"[MOVE] stop fully for '{object_name}'.")
            self.is_moving = False
            self.target_object_name = None
            # self.start_dance_sequence()

    # ─────────────────────────── LLM 호출 ────────────────────────────────
    def chat_once(self, messages, tools):
        payload = {
            "model": MODEL,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
            "temperature": 0.2,
            "top_p": 0.9,
            "max_tokens": 96,
            "stream": False
        }
        r = self.http.post(f"{RK_BASE}/api/chat", json=payload, timeout=15)
        r.raise_for_status()
        return r.json()


if __name__ == "__main__":
    RkllamaPi()
    rospy.spin()
