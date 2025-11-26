#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, requests, sys, re, time
import rospy
from std_msgs.msg import String
from yolo11_detect_pkg.msg import Yolo 
from sim2real_msg.msg import ControlState # sim2real_msg 토픽에 대한 가정
from geometry_msgs.msg import Twist
import numpy as np
from math import pi

RK_BASE = os.environ.get("RK_BASE", "http://127.0.0.1:8080")
MODEL = os.environ.get("RK_MODEL", "Qwen2.5-3B") 

class RkllamaPi:
    def __init__(self):
        self.TOOLS=[
            {
                "type": "function",
                "function": {
                    "name": "check_around",
                    "description": "로봇 주변에 보이는 모든 물체(예: 컵, 의자)의 목록을 확인하여 사용자에게 알려줄 때 사용합니다. (예: '주변에 뭐가 있어?')" 
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "go_to_object",
                    "description": "사용자가 지정한 특정 물체로 로봇을 이동시킬 때 사용합니다. 반드시 이동할 물체의 이름이 필요합니다.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "object_name": {
                                "type": "string",
                                "description": "이동할 대상 물체의 이름 (예: 'cup', 'table', 'person')"
                            }
                        },
                        "required": ["object_name"]
                    }
                },
            },
        ]
        
        rospy.init_node("rkllama_pi")

        self.usr_input = rospy.Subscriber("/gui/input", String, self.usr_input_cb)
        self.yolo_input = rospy.Subscriber("/yolo/detections", Yolo, self.yolo_input_cb)
        
        self.last_yolo_data = None 

        self.rkllama_output = rospy.Publisher("/rkllama/output", String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.joy_dance_pub=rospy.Publisher("/joy_msg", ControlState, queue_size=10)
        self.joy_msg=ControlState()
        
        # 추가된 비동기 이동 제어 상태 변수
        self.is_moving = False
        self.target_object_name = None
        self.is_aligned = False      # 정렬 완료 여부 플래그
        self.alignment_time = 0.0    # 정렬 완료된 시점의 시간 기록
        
        # 춤추기 상태 변수 추가
        self.is_dancing = False
        self.dance_end_time = 0.0
        
        # 10Hz (0.1초마다) 실행되는 ROS Timer 설정
        rospy.Timer(rospy.Duration(0.1), self.move_to_target_cb) 
    

    def yolo_input_cb(self, msg: Yolo):
        """YOLO 데이터를 받으면 최신 정보를 저장합니다."""
        self.last_yolo_data = msg

    def usr_input_cb(self, msg: String):
        user_input = msg.data
        rospy.loginfo(f"User Input: {user_input}")

        # --- (LLM 호출 및 도구 실행 로직은 변경 없음) ---
        base_system_prompt = (
            "You are a Lubancat ROS robot control agent. Your **sole function is to call a tool** "
            "when the user's request requires it, or reply with a **single, short, plain-text sentence** "
            "if no tool is needed. **DO NOT output markdown, lists, or any explanatory text.** "
            "For a query like 'what can you see', you **MUST** call the 'check_around' tool. "
            "For commands like 'go to X', you **MUST** call the 'go_to_object' tool. "
            "If the JSON tool call fails, only output 'GO_TO_OBJECT_NAME' or 'CHECK_AROUND' as plain text."
            "All responses must be strictly in **English or Korean**. Never use any other language, especially Chinese (Mandarin/Cantonese)."
        )

        messages = [
            {"role": "system", "content": base_system_prompt},
            {"role": "user", "content": user_input}
        ]
        
        function_name = None
        function_args = {}
        tool_call_requested = False
        is_go_to_command = user_input.lower().strip().startswith("go to ")

        try:
            # Phase 1: LLM 호출 및 응답 받기
            response_msg = self.chat_once(messages, self.TOOLS)
            messages.append(response_msg) 
            
            llm_raw_text = response_msg.get('message', {}).get('content', "No text content found.")
            rospy.loginfo(f"LLM Raw Text Response: {llm_raw_text}")


            if response_msg.get('tool_calls'):
                tool_call_requested = True
                rospy.loginfo("LLM requested Tool Call (JSON). Executing tool...")
                
                tool_call = response_msg['tool_calls'][0]
                function_name = tool_call['function']['name']
                
                try:
                    function_args = json.loads(tool_call['function']['arguments'])
                except json.JSONDecodeError:
                    rospy.logerr(f"JSONDecodeError: Failed to parse tool arguments for {function_name}. Using empty args.")
                    function_args = {}
                    
            elif is_go_to_command:
                rospy.logwarn(f"LLM failed to use JSON/Text Tool, but input was a 'go to' command. FORCING go_to_object call.")
                tool_call_requested = True
                function_name = "go_to_object"

            else:
                final_content = (llm_raw_text or "").strip()
                final_content_upper = final_content.upper().replace("#", "") 
                
                tool_names = {"CHECK_AROUND": "check_around", "GO_TO_OBJECT": "go_to_object"}
                
                if final_content_upper in tool_names or final_content_upper.startswith("GO_TO_"):
                    tool_call_requested = True
                    function_name = tool_names.get(final_content_upper, "go_to_object") 
                    if final_content_upper.startswith("GO_TO_"):
                        object_name = final_content_upper.replace("GO_TO_", "", 1).strip()
                        function_args = {"object_name": object_name}
                    else:
                        function_args = {}
                    rospy.logwarn(f"Model returned non-JSON text: {final_content}. Executing {function_name} fallback.")
                
                elif final_content:
                    rospy.loginfo(f"Final Response (Non-Tool): {final_content}")
                    self.rkllama_output.publish(final_content)
                    return 
                
                else:
                    rospy.logerr("LLM returned empty content or unparsable structure (e.g., 'none'). Terminating command.")
                    self.rkllama_output.publish("명령을 인식하지 못했습니다.")
                    return 

            
            # 도구 실행
            if not tool_call_requested:
                return 
            
            if function_name == "go_to_object":
                 if not function_args.get("object_name") and is_go_to_command:
                     potential_name = user_input[len("go to "):].strip()
                     if potential_name:
                          function_args["object_name"] = potential_name
                          rospy.logwarn(f"RECOVERY: Object name successfully extracted from user input: {potential_name}")


            rospy.loginfo(f"Executing Tool: {function_name}({function_args})")

            if function_name == "check_around":
                tool_result = self.execute_check_around()
            elif function_name == "go_to_object":
                # go_to_object는 이제 상태 설정만 합니다.
                tool_result = self.execute_go_to_object(function_args)
            else:
                tool_result = {"status": "error", "message": f"Unknown tool: {function_name}"}

            # 도구 실행 결과를 메시지에 추가
            messages.append({
                "role": "tool",
                "tool_call_id": "fallback_id" if not response_msg.get('tool_calls') else response_msg['tool_calls'][0]['id'],
                "content": json.dumps(tool_result, ensure_ascii=False)
            })
            
            # --- PHASE 2: 최종 응답 생성 ---
            phase2_messages = messages[:] 
            phase2_messages[0] = {
                "role": "system",
                "content": (
                    "You are a Lubancat ROS robot control agent. Based on the previous tool output, "
                    "provide a **single, short, plain-text sentence** to the user. "
                    "Do not output markdown, lists, or technical jargon. "
                    "All responses must be strictly in **English or Korean**. Never use any other language, especially Chinese (Mandarin/Cantonese). "
                    "Example: '주변에 컵과 책이 보입니다.' or '컵으로 이동을 시작합니다.' or 'I see a cup and a book.' or 'Starting movement to the cup.'"
                )
            }
            
            rospy.loginfo("Phase 2: Calling LLM with tool results for final response (Enhanced Prompt)...")
            final_response_msg = self.chat_once(phase2_messages, self.TOOLS) 
            
            final_content = final_response_msg.get('content') or final_response_msg.get('message', {}).get('content')
            final_content = (final_content or "LLM이 최종 응답을 생성하지 못했습니다.").strip()
            
            if final_content.upper().startswith("GO_TO_") or final_content.upper().startswith("CHECK_AROUND"):
                 final_content = tool_result.get("message", "요청 처리가 완료되었습니다.")

            rospy.loginfo(f"Final Response: {final_content}")
            self.rkllama_output.publish(final_content)

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"API Call Failed: {e}")
            self.rkllama_output.publish(f"LLM 서버 호출 오류: {e}")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")
            self.rkllama_output.publish(f"처리 중 예기치 않은 오류 발생: {e}")

    def execute_check_around(self):
        """'check_around' 도구의 실제 구현: 마지막 YOLO 데이터 기반으로 물체 목록 반환"""
        
        rospy.loginfo("[CHECK_AROUND] 도구 호출 시도.")
        if self.last_yolo_data is None or not self.last_yolo_data.detections:
            rospy.loginfo("[CHECK_AROUND] 물체 데이터 없음.")
            return {"status": "success", "objects_found": [], "message": "주변에 식별된 물체가 없습니다."}
        
        object_list = [(d.label, d.x_center, d.y_center) for d in self.last_yolo_data.detections]
        unique_object_names = sorted(list(set([d[0] for d in object_list])))
        
        if unique_object_names:
             msg = f"식별된 객체: {', '.join(unique_object_names)}"
        else:
             msg = "주변에 식별된 물체가 없습니다."
        
        rospy.loginfo(f"[CHECK_AROUND] 발견된 객체 목록: {unique_object_names}")
        return {"status": "success", "objects_found": unique_object_names, "message": msg}

    def execute_go_to_object(self, args):
        """ 변경: 이동 명령을 시작하고 제어권을 Timer 콜백으로 넘깁니다."""
        
        object_name = args.get("object_name")
        rospy.loginfo(f"[GO_TO] 요청 수신: object_name={object_name}") 
        
        if not object_name:
            return {"status": "error", "message": "이동할 물체의 이름이 지정되지 않았습니다."}

        if self.last_yolo_data is None or not self.last_yolo_data.detections:
            return {"status": "error", "message": f"현재 화면에서 '{object_name}'을(를) 찾을 수 없어 이동을 시작할 수 없습니다."}

        # 상태 변수 설정 및 이동 시작 시, is_aligned 초기화
        self.target_object_name = object_name
        self.is_moving = True
        self.is_aligned = False
        
        # 이전의 단발성 이동 명령을 중지
        self.cmd_vel_pub.publish(Twist())
        
        rospy.loginfo(f"[GO_TO] 이동 제어 시작. 목표: {object_name}. Timer가 제어를 인계받습니다.")

        return {"status": "success", "message": f"'{object_name}'(으)로 이동을 시작합니다."}

    def start_dance_sequence(self):
        """ 춤 동작 명령을 발행하고, 춤이 끝날 시간을 설정합니다."""
        
        # 이동 중에는 춤추기 불가 (move_to_target_cb에서 처리되지만 안전을 위해)
        if self.is_moving:
            return
            
        # 춤 동작 시간 설정 (예: 3초)
        DANCE_DURATION = 3.0 
        self.is_dancing = True
        self.dance_end_time = rospy.get_time() + DANCE_DURATION
        
        rospy.loginfo(f"[DANCE] 춤 동작 시작 명령 발행. {DANCE_DURATION}초간 대기합니다.")
        
        # 춤 동작 메시지 발행 (원래의 순서대로 발행)
        self.joy_msg.running_standby_switch=1.0
        self.joy_dance_pub.publish(self.joy_msg) # 1
        rospy.sleep(0.05)
        
        self.joy_msg.running_standby_switch=0.0
        self.joy_msg.candidate_left=1.0
        self.joy_msg.candidate_right=-1.0
        self.joy_msg.quit=1.0
        self.joy_msg.knee_angle_increase=1.0
        self.joy_dance_pub.publish(self.joy_msg) # 2 (실제 춤 동작)
        
        # 마지막 초기화 명령 (3)은 춤이 끝난 후 Timer에서 발행합니다.

    def move_to_target_cb(self, event):
        """ ROS 타이머에 의해 주기적으로 호출되며 지속적인 이동 제어를 수행합니다."""
        from geometry_msgs.msg import Twist
        
        # 춤 동작 중인지 최우선으로 확인합니다.
        if self.is_dancing:
            current_time = rospy.get_time()
            if current_time < self.dance_end_time:
                # 춤 동작이 끝날 때까지 이동 제어(Twist)를 발행하지 않고 대기합니다.
                rospy.loginfo(f"[DANCE] 춤 동작 대기 중. 남은 시간: {self.dance_end_time - current_time:.2f}s")
                return 
            else:
                # 춤 동작 완료
                rospy.loginfo("[DANCE] 춤 동작 완료. 상태 초기화 및 최종 정지 명령.")
                self.cmd_vel_pub.publish(Twist()) # 정지
                self.is_dancing = False
                self.is_moving = False # 이동 전체 종료
                self.target_object_name = None
                self.rkllama_output.publish("춤 동작을 마쳤습니다.") # 최종 사용자 응답

                # 춤 동작 메시지 최종 초기화 (3)
                self.joy_msg.running_standby_switch=1.0
                self.joy_msg.candidate_left=0.0
                self.joy_msg.candidate_right=0.0
                self.joy_msg.quit=0.0
                self.joy_msg.knee_angle_increase=0.0
                self.joy_dance_pub.publish(self.joy_msg)
                
                return # 초기화 후 함수 종료
        
        
        if not self.is_moving or self.target_object_name is None:
            return
            
        object_name = self.target_object_name
        
        # 1. 데이터 확인 및 목표 객체 검색
        if self.last_yolo_data is None or not self.last_yolo_data.detections:
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(f"[MOVE] YOLO 데이터 없음. '{object_name}' 이동 중단.")
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            return

        target_detection = None
        for d in self.last_yolo_data.detections:
            if d.label.lower() == object_name.lower():
                target_detection = d
                break

        if target_detection is None:
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(f"[MOVE] 목표 '{object_name}'을(를) 놓쳤습니다. 이동 중단.")
            self.is_moving = False
            self.target_object_name = None
            self.is_aligned = False
            return

        # 3. 제어 파라미터 및 오차 계산
        LINEAR_SPEED = 0.35              # 직진 속도 상한
        MIN_LINEAR_SPEED = 0.2           # 회전 중 최소 전진 속도 (Deadband 회피)
        PIXEL_TOLERANCE = 50 
        FINAL_STOP_Y_CENTER = 400 
        ALIGNMENT_HOLD_TIME = 0.8        # 직진 강제 유지 시간

        cx = target_detection.x_center
        image_width = getattr(self.last_yolo_data, 'image_width', 640)
        center_x = image_width / 2.0
        error_x = center_x - cx # 픽셀 오차 (왼쪽 회전 시 양수, 오른쪽 회전 시 음수)
        
        # 4. Twist 메시지 계산
        twist_msg = Twist()
        calculated_angular_z = error_x * (pi / image_width)
        should_stop = False

        
        # ----------------------------------------------------
        # A. 최종 정지 조건 검사 (최우선)
        if target_detection.y_center > FINAL_STOP_Y_CENTER: 
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            should_stop = True
            self.is_aligned = False 
            rospy.loginfo("[MOVE] 목표 근접 도달. 최종 정지 명령.")
            
        # B. 정렬 강제 유지 조건 검사
        elif self.is_aligned:
            current_time = rospy.get_time()
            if current_time - self.alignment_time < ALIGNMENT_HOLD_TIME:
                # 정렬 직후, 강제로 직진 명령을 유지
                twist_msg.linear.x = LINEAR_SPEED
                twist_msg.angular.z = 0.0
                rospy.loginfo(f"[MOVE] 정렬 강제 유지 중 ({current_time - self.alignment_time:.2f}s).")
            else:
                # 유지 시간이 지나면 일반 추적 모드로 복귀
                self.is_aligned = False 
                rospy.loginfo("[MOVE] 강제 유지 종료. 일반 추적 복귀.")


        # C. 정렬 완료 (직진 시작) 조건 검사
        elif abs(error_x) < PIXEL_TOLERANCE:
            
            # 정렬 완료가 감지되면, 강제 유지 상태로 전환
            self.is_aligned = True
            self.alignment_time = rospy.get_time()
            
            # 이 틱에서는 직진 명령을 발행하고, 다음 틱부터 강제 유지 시작
            twist_msg.linear.x = LINEAR_SPEED
            twist_msg.angular.z = 0.0
            rospy.loginfo("[MOVE] 정렬 완료 감지! 직진 및 강제 유지 시작.")
            
        # D. 정렬 중 조건 검사 
        else: 
            # 회전 중에도 최소 속도 이상으로 전진 (Deadband 회피)
            twist_msg.linear.x = MIN_LINEAR_SPEED
            twist_msg.angular.z = np.clip(calculated_angular_z, -0.6, 0.6)
            print(f"@@@@@@@@@조정중!!!!!! error_x: {error_x}, angular.z: {twist_msg.angular.z}, linear.x: {twist_msg.linear.x}") # 사용자 피드백 반영
            rospy.loginfo(f"[MOVE] 일반 정렬 중. L:{twist_msg.linear.x:.2f}, A:{twist_msg.angular.z:.2f}")

        # 5. cmd_vel 발행
        self.cmd_vel_pub.publish(twist_msg)
        
        # 6. 정지 명령을 내렸으면 이동 상태를 종료하고 춤 동작을 시작합니다.
        if should_stop:
            rospy.loginfo(f"[MOVE] 목표 '{object_name}' 근접 도달. 이동 제어를 완전히 종료합니다.")
            self.is_moving = False
            self.target_object_name = None
            
            # 춤 동작 시작
            # self.start_dance_sequence()
            
            # move_to_target_cb의 다음 호출부터 is_dancing 로직이 제어를 인계받습니다.


    def chat_once(self, messages, tools):
        """rkllama API를 호출하고 응답을 JSON으로 반환"""
        payload = {
            "model": MODEL,
            "messages": messages,
            "tools": tools,
            "stream": False # JSONDecodeError 방지를 위해 스트리밍 비활성화
        }

        r=requests.post(f"{RK_BASE}/api/chat", json=payload, timeout=120) 
        
        r.raise_for_status()
        return r.json()


if __name__=="__main__":
    RkllamaPi()
    rospy.spin()