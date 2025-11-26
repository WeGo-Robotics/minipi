#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from yolo11_detect_pkg.msg import Yolo
from math import pi

# ---- 상태 상수 ----
RUNNING = 0  # 상태 토픽 발행용

# ⭐️ 6단계 순차적 FSM (지시사항 기반) ⭐️
STG_INIT_MEMORIZE  = 1 # 1) 초기 공/골대 좌표 저장
STG_APPROACH_ALIGN = 2 # 2) 공을 향해 정렬 (X 좌표 기반)
STG_APPROACH_DRIVE = 3 # 3) 공 근처로 전진 (Y 좌표 기반)
STG_KICK_ALIGN     = 4 # 4) 최종 정렬: 골대 검색 및 정렬 (angular.z + linear.y)
STG_KICK_DRIVE     = 5 # 5) 킥 전진 (3초 고정)
STG_FINISHED       = 6 # 6) 종료 상태

# YOLO 실행/정지 플래그 값
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0

# ⭐️ 동작 임계값 및 상수 ⭐️
ALIGN_DEADBAND_PX    = 50     # STG 2 정렬 완료 픽셀 오차
KICK_ALIGN_DEADBAND_PX = 50   # STG 4 최종 정렬 픽셀 오차
APPROACH_DRIVE_Y_THRESHOLD = 350 # 공이 화면 하단 350px에 오면 정지 
KICK_DRIVE_TICKS     = 30     # 3.0초 킥 전진 시간 (10Hz 기준)

# 발 위치 기반 상수
FOOT_OFFSET_PX = 37 # 공 중심이 카메라 중앙에서 37px 오른쪽으로 치우쳐야 발에 맞음
LATERAL_SPEED_MAX = 0.5 # linear.y 최대 속도 (좌우 이동)


class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # 퍼블리셔
        self.twist_pub = None
        self.ensure_twist_pub() # /cmd_vel 토픽으로 퍼블리셔 생성

        self.state_pub   = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.control_pub = rospy.Publisher("/yolo_run_control", Int8, queue_size=1)
        self.pi_mode     = rospy.Publisher("/pi_mode", String, queue_size=10)

        # 서브스크라이버
        self.detections_topic = rospy.get_param("~detections_topic", "/YoloInfo")
        rospy.Subscriber(self.detections_topic, Yolo, self.yolo_cb)

        # 이미지 크기
        self.img_w, self.img_h = 640, 480
        self.cx = self.img_w * 0.5 # 320.0

        # 타이밍/동작 파라미터
        self.tick_hz    = rospy.get_param("~tick_hz", 10)
        self.lost_limit = rospy.get_param("~lost_limit_ticks", 30)

        # 선속도/각속도 상수
        self.v_approach = rospy.get_param("~v_approach", 0.3)
        self.v_kick = rospy.get_param("~v_kick", 0.5)
        self.w_max      = rospy.get_param("~w_max", 0.8)

        # ⭐️ 내부 상태 ⭐️
        self.stage = STG_INIT_MEMORIZE
        self._ticks_since_seen = self.lost_limit + 1
        self.twist_enabled     = True
        
        # ⭐️ 초기 좌표 메모리 (1단계에서 저장) ⭐️
        self.ball_init = None 
        self.goal_init = None 
        
        self._kick_drive_left = 0
        self.cmd_twist = Twist() 
        self.stage_finished = False
        
        # ⭐️ STG 4 검색용 상수 ⭐️
        self.goal_search_wz = self.w_max * 0.5 # 골대 검색을 위한 느린 회전 속도 (0.4)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self.loop)
        rospy.loginfo("Publishers ready: /cmd_vel, /yolo_state, /yolo_run_control")

    # ---------- 유틸 ----------
    def ensure_twist_pub(self):
        """ Twist 퍼블리셔를 /cmd_vel 토픽으로 생성/재생성합니다. """
        if self.twist_pub is None:
            self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
            self.twist_enabled = True
            rospy.loginfo("Twist publisher (re)created on /cmd_vel.")

    def disable_twist_publishing(self):
        if self.twist_pub is not None:
            try:
                self.twist_pub.unregister()
            except Exception as e:
                rospy.logerr(f"Publisher unregister error: {e}")
            self.twist_pub = None
        self.twist_enabled = False
        rospy.logwarn("CMD publishing disabled.")

    def _calculate_angular_z(self, target_x, deadband_px):
        """X 좌표를 중앙에 맞추기 위한 angular.z 계산. target_x는 공/골대/중심점."""
        # 목표는 카메라 중앙 (self.cx)
        e_px = target_x - self.cx # 오차가 양수: 목표가 중앙보다 오른쪽
        
        if abs(e_px) <= deadband_px:
            return 0.0, True # 정렬 완료
        
        # P 제어 기반 각속도
        w_cmd = e_px * (self.w_max / (self.img_w / 2.0))
        # 왼쪽이 양수이므로, 오차가 양수일 때(-w) 오른쪽 회전 명령
        wz = float(np.clip(-w_cmd, -self.w_max, self.w_max)) 
        
        return wz, False

    def _calculate_linear_y_for_kick(self, target_x_for_foot, deadband_px):
        """linear.y를 사용하여 로봇 발의 X 위치를 목표 지점(target_x_for_foot)에 맞춤"""
        # 로봇 발의 x_center 위치를 고려한 목표 지점
        foot_x_position = self.cx + FOOT_OFFSET_PX
        
        # 발 위치와 목표 지점의 오차 (오차가 양수: 목표가 발보다 오른쪽)
        e_px = target_x_for_foot - foot_x_position 

        if abs(e_px) <= deadband_px:
            return 0.0, True # 정렬 완료

        # P 게인 설정 (이전 테스트에서 사용된 값)
        LATERAL_P_GAIN = 0.005 
        linear_y_cmd = e_px * LATERAL_P_GAIN
        
        ly = float(np.clip(linear_y_cmd, -LATERAL_SPEED_MAX, LATERAL_SPEED_MAX))
        
        return ly, False


    def _set_next_command(self, next_stage):
        """ 다음 단계로 전이할 때 현재 명령을 정지하고 새 상태를 설정합니다. """
        self.cmd_twist = Twist() 
        self.stage = next_stage
        rospy.loginfo(f"[Transition] Moving to STG {next_stage}")
        
        # 5단계(킥) 진입 시 Ticks 초기화
        if next_stage == STG_KICK_DRIVE:
            self._kick_drive_left = KICK_DRIVE_TICKS
            self.cmd_twist.linear.x = self.v_kick # 킥 시작 명령
        
        # 6단계(종료) 진입 시 정지
        elif next_stage == STG_FINISHED:
            self.stage_finished = True
            self.disable_twist_publishing()

    # ---------- 메인 루프 (KICK_DRIVE 실행 및 명령 발행) ----------
    def loop(self, _):
        # ⭐️ 5단계 KICK_DRIVE (시간 기반) 실행 및 카운트다운 ⭐️
        if self.stage == STG_KICK_DRIVE:
            if self._kick_drive_left > 0:
                self._kick_drive_left -= 1
                self.pi_mode.publish(f"STG 5: KICKING... left: {self._kick_drive_left}")
            
            if self._kick_drive_left == 0 and not self.stage_finished:
                self._set_next_command(STG_FINISHED)

        # ⭐️ Twist 발행 (FINISH 전까지) ⭐️
        if self.twist_enabled and self.twist_pub is not None and not self.stage_finished and self.stage != STG_INIT_MEMORIZE:
            self.twist_pub.publish(self.cmd_twist)
        
        # ⭐️ 워치독 및 YOLO 실행 유지 (생략 가능) ⭐️
        self._ticks_since_seen += 1
        if self._ticks_since_seen > self.lost_limit and self.twist_enabled and not self.stage_finished:
            # 워치독: 명령 무발행 후 초기 단계로 복귀
            self.cmd_twist = Twist() # 정지 명령
            self.twist_pub.publish(self.cmd_twist)
            rospy.logwarn("Target lost. Returning to STG_INIT_MEMORIZE.")
            self.stage = STG_INIT_MEMORIZE
            self.ball_init = None
            self.goal_init = None
            
        self.control_pub.publish(Int8(YOLO_CONTROL_RUN))
        self.state_pub.publish(Int8(RUNNING))


    # ---------- 콜백 (Sensing 기반 FSM 제어) ----------
    def yolo_cb(self, msg: Yolo):
        """검출 결과에 따라 FSM 전이 및 명령을 계산합니다."""
        
        self._ticks_since_seen = 0 # 공/골대를 봤으니 워치독 리셋
        
        det_ball = next((d for d in msg.detections if (d.label or "").lower() == "ball"), None)
        det_goal = next((d for d in msg.detections if (d.label or "").lower() == "goal"), None)

        if self.stage_finished:
            self.pi_mode.publish("STG 6: FINISHED.")
            return

        # ⭐️ 1. STG_INIT_MEMORIZE: 초기 좌표 저장 ⭐️
        if self.stage == STG_INIT_MEMORIZE:
            if det_ball is not None:
                self.ball_init = (det_ball.x_center, det_ball.y_center)
                # 골대 초기 좌표 저장 (STG 4의 최종 목표 계산에 사용)
                self.goal_init = (det_goal.x_center, det_goal.y_center) if det_goal is not None else None 
                self._set_next_command(STG_APPROACH_ALIGN) 
                rospy.loginfo("STG 1: Ball detected. Proceeding to STG 2.")
            else:
                self.pi_mode.publish("STG 1: Waiting for Ball.")
            return

        # 이후 단계는 반드시 공 검출이 있어야 명령 계산 가능
        if det_ball is None:
            self.cmd_twist = Twist() # 공이 사라지면 정지
            self.pi_mode.publish(f"STG {self.stage}: Ball lost. Stopping.")
            return

        # ⭐️ 2. STG_APPROACH_ALIGN: 공과 X 좌표 정렬 ⭐️
        if self.stage == STG_APPROACH_ALIGN:
            wz, is_aligned = self._calculate_angular_z(det_ball.x_center, ALIGN_DEADBAND_PX)
            
            self.cmd_twist.angular.z = wz
            self.cmd_twist.linear.x = 0.0 # 전진 정지
            self.cmd_twist.linear.y = 0.0 # 좌우 이동 정지
            self.pi_mode.publish(f"STG 2: Aligning to Ball. wz={wz:.2f}")

            if is_aligned:
                self._set_next_command(STG_APPROACH_DRIVE)
                rospy.loginfo("STG 2: Ball aligned. Proceeding to STG 3.")
            return

        # ⭐️ 3. STG_APPROACH_DRIVE: 공 근처로 전진 (Y 좌표 기반) ⭐️
        if self.stage == STG_APPROACH_DRIVE:
            if det_ball.y_center >= APPROACH_DRIVE_Y_THRESHOLD: # 공이 충분히 가까워지면
                self._set_next_command(STG_KICK_ALIGN) # 전이
                rospy.loginfo("STG 3: Reached Ball proximity. Proceeding to STG 4.")
                return
            
            self.cmd_twist.linear.x = self.v_approach
            self.cmd_twist.angular.z = 0.0 # 회전 정지
            self.cmd_twist.linear.y = 0.0 # 좌우 이동 정지
            self.pi_mode.publish(f"STG 3: Driving to Ball. y={det_ball.y_center:.0f}")
            return
            
        # ⭐️ 4. STG_KICK_ALIGN: 최종 정렬 (골대 검색/사용) ⭐️
        if self.stage == STG_KICK_ALIGN:
            self.cmd_twist.linear.x = 0.0 # 전진 정지
            
            # --- 4.1. 골대가 안 보일 때: 공 중심으로 회전하며 골대 검색 ---
            if det_goal is None:
                # 공의 X 좌표를 카메라 중앙에 맞추면서 회전
                wz, is_yaw_aligned = self._calculate_angular_z(det_ball.x_center, ALIGN_DEADBAND_PX)
                
                # 공이 중앙에 있다면, 골대를 찾기 위해 느린 속도로 강제 회전
                if is_yaw_aligned:
                    # 회전 방향은 임의로 설정 (예: 시계 방향 -)
                    wz = -self.goal_search_wz 
                    self.pi_mode.publish(f"STG 4: Searching Goal. Ball aligned, forced rotation {wz:.2f}.")
                else:
                    self.pi_mode.publish(f"STG 4: Searching Goal. Aligning to Ball wz={wz:.2f}.")
                
                self.cmd_twist.angular.z = wz
                self.cmd_twist.linear.y = 0.0 # 좌우 이동 정지
                return

            # --- 4.2. 골대가 보일 때: 골대와 공에 맞춰 정렬 ---
            else:
                # A. Angular.z: 골대 X 좌표를 중앙에 맞춰 방향 보정 (골대가 이미지 중앙과 비슷하도록)
                wz, is_yaw_aligned = self._calculate_angular_z(det_goal.x_center, KICK_ALIGN_DEADBAND_PX) 
                
                # B. Linear.y: 공-골대 중심을 발 위치에 맞춰 좌우 이동
                target_x_for_foot = (det_ball.x_center + det_goal.x_center) / 2.0
                ly, is_lateral_aligned = self._calculate_linear_y_for_kick(target_x_for_foot, KICK_ALIGN_DEADBAND_PX)
                
                self.cmd_twist.angular.z = wz
                self.cmd_twist.linear.y = ly
                
                self.pi_mode.publish(f"STG 4: Goal Found. wz={wz:.2f}, ly={ly:.2f}. Yaw Aligned:{is_yaw_aligned}, Lat Aligned:{is_lateral_aligned}")
                
                # 최종 정렬 조건: 골대 중앙 정렬 및 발 위치 정렬 모두 만족
                if is_yaw_aligned and is_lateral_aligned:
                    self._set_next_command(STG_KICK_DRIVE)
                    rospy.loginfo("STG 4: Final alignment complete. Proceeding to STG 5 (Kick).")
                return

        # ⭐️ 5. STG_KICK_DRIVE: 킥 (Time-based, loop에서 처리) ⭐️
        if self.stage == STG_KICK_DRIVE:
            self.pi_mode.publish(f"STG 5: Kicking. x={self.v_kick:.1f} for {KICK_DRIVE_TICKS/self.tick_hz:.1f}s.")
            return

        # ⭐️ 6. STG_FINISHED ⭐️
        if self.stage == STG_FINISHED:
            self.pi_mode.publish("STG 6: FINISHED. Robot stopped.")


if __name__ == "__main__":
    BallTrack()
    rospy.spin()