#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rkllama_conv.py (model-agnostic, ROS1)

Design goals:
- Tool decision is rule-based (no LLM tool_calls dependency)
- LLM is only a text generator (no tools/system/multi-role history)
- GUI streaming markers are always guaranteed: __LLM_START__ / __LLM_END__
- Robust against /api/chat streaming format differences
"""

import os
import json
import time
import threading
import requests

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi
import numpy as np

from yolo11_detect_pkg.msg import Yolo
from sim2real_msg.msg import Joy  # adjust if your actual msg differs


RK_BASE = os.environ.get("RK_BASE", "http://127.0.0.1:8080")
MODEL = os.environ.get("RK_MODEL", "gemma2:2b")  # gemma2:2b or qwen2.5:3b etc.


# -------------------------------
# Utilities
# -------------------------------
def _safe_json_loads(s: str):
    try:
        return json.loads(s)
    except Exception:
        return None


def _extract_stream_payload_line(raw_line: bytes) -> str:
    """
    rkllama sometimes streams like:
      b'data: {...json...}'
    or just:
      b'{...json...}'
    """
    if not raw_line:
        return ""
    line = raw_line.strip()
    if line.startswith(b"data:"):
        line = line[len(b"data:") :].strip()
    try:
        return line.decode("utf-8", errors="replace")
    except Exception:
        return ""


def _extract_text_from_chunk(chunk: dict) -> str:
    """
    Be flexible: different servers/models may put text in different fields.
    """
    if not isinstance(chunk, dict):
        return ""

    # most common:
    msg = chunk.get("message")
    if isinstance(msg, dict):
        c = msg.get("content")
        if isinstance(c, str) and c:
            return c

    # some servers:
    c = chunk.get("response")
    if isinstance(c, str) and c:
        return c

    # some may do:
    c = chunk.get("content")
    if isinstance(c, str) and c:
        return c

    return ""


# -------------------------------
# Main Node
# -------------------------------
class RkllamaPi:
    def __init__(self):
        rospy.init_node("rkllama_pi", anonymous=False)

        # ROS IO
        self.usr_input = rospy.Subscriber("/gui/input", String, self.usr_input_cb)
        self.yolo_input = rospy.Subscriber("/yolo/detections", Yolo, self.yolo_input_cb)

        self.rkllama_output = rospy.Publisher("/rkllama/output", String, queue_size=50)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.joy_pub = rospy.Publisher("/joy_msg", Joy, queue_size=10)

        # Perception cache
        self.last_yolo_data = None

        # Movement state
        self.is_moving = False
        self.target_object_name = None
        self.is_aligned = False
        self.alignment_time = 0.0

        # Optional action state (dance etc.)
        self.is_dancing = False
        self.dance_end_time = 0.0
        self.joy_msg = Joy()

        # Timer for motion control
        rospy.Timer(rospy.Duration(0.1), self.move_to_target_cb)

        # Streaming control (prevent overlapping streams)
        self._req_lock = threading.Lock()
        self._active_request_id = 0  # increments each user input
        self._streaming = False

        rospy.loginfo("[rkllama_conv] node ready. RK_BASE=%s MODEL=%s", RK_BASE, MODEL)

    # -------------------------------
    # ROS Callbacks
    # -------------------------------
    def yolo_input_cb(self, msg: Yolo):
        self.last_yolo_data = msg

    def usr_input_cb(self, msg: String):
        user_input = (msg.data or "").strip()
        if not user_input:
            return

        rospy.loginfo("[USER] %s", user_input)

        # cancel any ongoing stream by bumping request id
        with self._req_lock:
            self._active_request_id += 1
            req_id = self._active_request_id

        # 1) Rule-based intent routing (NO LLM for tool decision)
        intent, args = self.parse_intent(user_input)

        if intent == "check_around":
            self.publish_bubble(req_id, lambda: self.execute_check_around()["message"])
            return

        if intent == "go_to_object":
            self.publish_bubble(req_id, lambda: self.execute_go_to_object(args)["message"])
            return

        # 2) Otherwise: "normal chat" → LLM text generator only
        t = threading.Thread(
            target=self.llm_stream_chat,
            args=(req_id, user_input),
            daemon=True,
        )
        t.start()

    # -------------------------------
    # Intent (model-independent)
    # -------------------------------
    def parse_intent(self, text: str):
        t = (text or "").strip()
        lower = t.lower()

        # check_around keywords
        if any(
            k in lower
            for k in [
                "주변",
                "보여",
                "보이는",
                "뭐 있어",
                "뭐있어",
                "뭐가 있어",
                "뭐가있어",
                "what can you see",
                "what is around",
                "around you",
                "find objects",
                "see around",
            ]
        ):
            return ("check_around", {})

        # go_to patterns (Korean + English)
        # e.g., "go to cup", "컵으로 가", "컵으로 가줘", "cup로 가"
        if lower.startswith("go to "):
            name = t[6:].strip()
            return ("go_to_object", {"object_name": name})

        # very simple Korean patterns
        # "XX로 가", "XX으로 가", "XX로 이동", "XX으로 이동"
        for suffix in ["로 가", "으로 가", "로가", "으로가", "로 이동", "으로 이동", "로이동", "으로이동"]:
            if suffix in t:
                name = t.split(suffix)[0].strip()
                if name:
                    return ("go_to_object", {"object_name": name})

        return ("chat", {})

    # -------------------------------
    # GUI Bubble helpers (START/END guaranteed)
    # -------------------------------
    def publish_bubble(self, req_id: int, producer_fn):
        """
        Runs producer_fn() and publishes it as one bubble,
        always wrapping with START/END, and respecting cancellation.
        """
        self._publish_start(req_id)
        try:
            if not self._is_request_active(req_id):
                return
            text = producer_fn() or ""
            if self._is_request_active(req_id):
                self.rkllama_output.publish(text)
        except Exception as e:
            rospy.logerr("[publish_bubble] error: %r", e)
            if self._is_request_active(req_id):
                self.rkllama_output.publish("(처리 중 오류)")
        finally:
            self._publish_end(req_id)

    def _publish_start(self, req_id: int):
        if not self._is_request_active(req_id):
            return
        self._streaming = True
        self.rkllama_output.publish("__LLM_START__")

    def _publish_end(self, req_id: int):
        if not self._is_request_active(req_id):
            return
        self.rkllama_output.publish("__LLM_END__")
        self._streaming = False

    def _is_request_active(self, req_id: int) -> bool:
        with self._req_lock:
            return req_id == self._active_request_id

    # -------------------------------
    # Tools (no LLM involvement)
    # -------------------------------
    def execute_check_around(self):
        rospy.loginfo("[TOOL] check_around")
        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", None):
            return {"status": "success", "objects_found": [], "message": "주변에 식별된 물체가 없습니다."}

        detections = self.last_yolo_data.detections
        names = sorted(set([d.label for d in detections if getattr(d, "label", "")]))
        if names:
            return {"status": "success", "objects_found": names, "message": f"식별된 객체: {', '.join(names)}"}
        return {"status": "success", "objects_found": [], "message": "주변에 식별된 물체가 없습니다."}

    def execute_go_to_object(self, args):
        rospy.loginfo("[TOOL] go_to_object args=%s", args)
        object_name = (args or {}).get("object_name", "").strip()
        if not object_name:
            return {"status": "error", "message": "이동할 물체의 이름이 지정되지 않았습니다."}

        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", None):
            return {"status": "error", "message": f"현재 화면에서 '{object_name}'을(를) 찾을 수 없어 이동을 시작할 수 없습니다."}

        # Start motion control loop
        self.target_object_name = object_name
        self.is_moving = True
        self.is_aligned = False
        self.cmd_vel_pub.publish(Twist())  # clear any previous cmd

        return {"status": "success", "message": f"'{object_name}'(으)로 이동을 시작합니다."}

    # -------------------------------
    # Motion control (unchanged essence)
    # -------------------------------
    def move_to_target_cb(self, event):
        if self.is_dancing:
            # (optional) keep if you use dance logic
            return

        if not self.is_moving or not self.target_object_name:
            return

        if self.last_yolo_data is None or not getattr(self.last_yolo_data, "detections", None):
            self.cmd_vel_pub.publish(Twist())
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            rospy.logwarn("[MOVE] no YOLO data -> stop")
            return

        obj = self.target_object_name
        target = None
        for d in self.last_yolo_data.detections:
            if getattr(d, "label", "").lower() == obj.lower():
                target = d
                break

        if target is None:
            self.cmd_vel_pub.publish(Twist())
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            rospy.logwarn("[MOVE] lost target '%s' -> stop", obj)
            return

        LINEAR_SPEED = 0.35
        MIN_LINEAR_SPEED = 0.2
        PIXEL_TOL = 50
        FINAL_STOP_Y = 400
        ALIGN_HOLD = 0.8

        image_width = getattr(self.last_yolo_data, "image_width", 640)
        cx = getattr(target, "x_center", image_width / 2.0)
        err_x = (image_width / 2.0) - cx
        ang = float(err_x) * (pi / float(image_width))
        ang = float(np.clip(ang, -0.6, 0.6))

        twist = Twist()
        should_stop = False

        if getattr(target, "y_center", 0) > FINAL_STOP_Y:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            should_stop = True

        elif self.is_aligned:
            now = rospy.get_time()
            if now - self.alignment_time < ALIGN_HOLD:
                twist.linear.x = LINEAR_SPEED
                twist.angular.z = 0.0
            else:
                self.is_aligned = False

        elif abs(err_x) < PIXEL_TOL:
            self.is_aligned = True
            self.alignment_time = rospy.get_time()
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = 0.0

        else:
            twist.linear.x = MIN_LINEAR_SPEED
            twist.angular.z = ang

        self.cmd_vel_pub.publish(twist)

        if should_stop:
            self.cmd_vel_pub.publish(Twist())
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False

    # -------------------------------
    # LLM (model-agnostic)
    # -------------------------------
    def llm_stream_chat(self, req_id: int, user_text: str):
        """
        Model-agnostic:
        - Single "user" message only (no system/tools/history) to avoid:
          "Conversation roles must alternate ..." template errors
        - If streaming fails, fallback to non-stream once.
        - Always guarantees START/END.
        """
        self._publish_start(req_id)

        # minimalist prompt wrapper: keeps the model on-track without system role
        prompt = "Reply in Korean or English in 1-2 short sentences. " "Do not use markdown or lists.\n\n" f"User: {user_text}\nAssistant:"

        messages = [{"role": "user", "content": prompt}]

        try:
            # 1) try streaming
            ok = self._rk_stream(req_id, messages)
            if ok:
                return

            # 2) fallback once (non-stream)
            if self._is_request_active(req_id):
                text = self._rk_once(messages)
                if text:
                    self.rkllama_output.publish(text)

        except Exception as e:
            rospy.logerr("[llm_stream_chat] error: %r", e)
            if self._is_request_active(req_id):
                self.rkllama_output.publish("(LLM 오류)")
        finally:
            self._publish_end(req_id)

    def _rk_stream(self, req_id: int, messages) -> bool:
        payload = {
            "model": MODEL,
            "messages": messages,
            "stream": True,
        }

        try:
            with requests.post(
                f"{RK_BASE}/api/chat",
                json=payload,
                stream=True,
                timeout=120,
            ) as r:
                if r.status_code != 200:
                    rospy.logerr("[rk_stream] http %s", r.status_code)
                    return False

                for raw_line in r.iter_lines():
                    if not self._is_request_active(req_id):
                        return True  # canceled, treat as handled

                    line = _extract_stream_payload_line(raw_line)
                    if not line:
                        continue

                    chunk = _safe_json_loads(line)
                    if not chunk:
                        continue

                    if chunk.get("done") is True:
                        return True

                    txt = _extract_text_from_chunk(chunk)
                    if txt:
                        # minor cleanup (optional)
                        txt = txt.replace("#", "")
                        self.rkllama_output.publish(txt)

                return True
        except Exception as e:
            rospy.logerr("[rk_stream] exception: %r", e)
            return False

    def _rk_once(self, messages) -> str:
        payload = {
            "model": MODEL,
            "messages": messages,
            "stream": False,
        }
        r = requests.post(f"{RK_BASE}/api/chat", json=payload, timeout=120)
        if r.status_code != 200:
            rospy.logerr("[rk_once] http %s body=%s", r.status_code, r.text[:2000])
            return ""

        data = r.json()
        # common:
        msg = data.get("message")
        if isinstance(msg, dict):
            c = msg.get("content")
            if isinstance(c, str):
                return c.replace("#", "").strip()

        # fallback:
        c = data.get("response")
        if isinstance(c, str):
            return c.replace("#", "").strip()

        return ""


if __name__ == "__main__":
    RkllamaPi()
    rospy.spin()
