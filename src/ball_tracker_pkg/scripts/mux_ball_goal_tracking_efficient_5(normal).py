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

# 단계 FSM: 1) linear.y 정렬 → 2) angular.z 각도 정렬 → 3) linear.x 전진
STG_ALIGN_LATERAL = 1
STG_ALIGN_YAW     = 2
STG_DRIVE_FORWARD = 3

# YOLO 실행/정지 플래그 값
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0

# pixel to distance 인자 (y기준)
LINEAR_COEF   = (-0.010895, 6.024430)
RATIONAL_COEF = (-84.166458, 50689.309424, 25.534673, 1)

# Goal Alignment 상수
GOAL_ALIGN_DEADBAND_PX = 100  # |goal.x - ball.x| <= 이면 정렬 완료로 간주
LATERAL_SPEED_MAX      = 0.5  # linear.y (왼쪽 +)

# 발 위치 기반 상수
FOOT_OFFSET_PX = 20 # 공 중심이 카메라 중앙에서 37px 오른쪽으로 치우쳐야 발에 맞음
YAW_ALIGN_DEADBAND_PX = 10 # 픽셀 오차 데드밴드 (정렬 완료 기준)


class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # 퍼블리셔
        self.twist_pub = None
        self.ensure_twist_pub()  # /cmd_vel/auto 퍼블리셔 생성

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
        self.tick_hz    = rospy.get_param("~tick_hz", 10)              # 제어 주기 Hz
        self.lost_limit = rospy.get_param("~lost_limit_ticks", 30)     # 공 미검출 연속 틱(=3s) 넘으면 무발행
        self.stop_immediately_if_lost = rospy.get_param("~stop_immediately_if_lost", False)

        # 선속도 매핑(linear.x)
        self.v_min = rospy.get_param("~v_min", 0.1)
        self.v_max = rospy.get_param("~v_max", 0.3)

        # 각속도 매핑(angular.z)  (왼쪽 회전 +, 오른쪽 회전 -)
        self.w_max      = rospy.get_param("~w_max", 0.8)
        self.w_deadband = rospy.get_param("~w_deadband", 0.10)
        self.w_gamma    = rospy.get_param("~w_gamma", 5.0)
        self.w_offset   = rospy.get_param("~w_offset", 0.0)
        self.w_dir      = rospy.get_param("~w_dir", -1.0)  # e_px>0(공이 오른쪽) -> 음수 회전(오른쪽)
        self.w_gain     = rospy.get_param("~w_gain", 1.0)
        self.w_min      = rospy.get_param("~w_min", 0.0)

        # 2단계(YAW 정밀 정렬) (사용하지 않음, 하지만 파라미터는 유지)
        self.yaw_deadband_px      = rospy.get_param("~yaw_deadband_px", 20)
        self.yaw_fine_deadband_px = rospy.get_param("~yaw_fine_deadband_px", 10)
        self.yaw_kp               = rospy.get_param("~yaw_kp", 0.004)
        self.yaw_w_min            = rospy.get_param("~yaw_w_min", 0.05)

        # 드라이브 종료(옵션)
        self.stop_distance_m = rospy.get_param("~stop_distance_m", 0.25)

        # 내부 상태
        self._ticks_since_seen = self.lost_limit + 1
        self.twist_enabled     = True
        self.stage = STG_ALIGN_LATERAL

        # 주기 타이머 (워치독 및 상태/제어 신호 발행)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self.loop)

        rospy.loginfo("Subscribed detections: %s", rospy.resolve_name(self.detections_topic))
        rospy.loginfo("Publishers ready: /cmd_vel/auto, /yolo_state, /yolo_run_control")

    # ---------- 유틸 ----------
    def ensure_twist_pub(self):
        """필요시 /cmd_vel 퍼블리셔 재생성 및 발행 허용"""
        if self.twist_pub is None:
            self.twist_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
            self.twist_enabled = True
            rospy.loginfo("Twist publisher (re)created.")

    def disable_twist_publishing(self):
        """
        /cmd_vel 무발행 강제: 퍼블리셔 해제 및 논리 플래그 비활성화
        (twist_mux의 timeout에 걸려 joy로 스위칭되도록)
        """
        if self.twist_pub is not None:
            try:
                self.twist_pub.unregister()
            except Exception as e:
                rospy.logerr(f"Publisher unregister error: {e}")
            self.twist_pub = None
        self.twist_enabled = False
        rospy.logwarn("CMD publishing disabled → /cmd_vel/auto 무발행(타임아웃 유도).")

    @staticmethod
    def pixel_to_distance(y_pix, method):
        if len(method) == 2:
            return method[0] * y_pix + method[1]
        else:
            return (method[0] * y_pix + method[1]) / (method[2] * y_pix + method[3])

    # ---------- 메인 루프 (워치독 및 YOLO 실행 유지) ----------
    def loop(self, _):
        # 공 미검출 경과 시간 증가
        self._ticks_since_seen += 1

        # 미검출 연속시간이 임계치 초과 → /cmd_vel 무발행(워치독으로 정지 유도)
        if self._ticks_since_seen > self.lost_limit and self.twist_enabled:
            self.disable_twist_publishing()
            self.stage = STG_ALIGN_LATERAL  # 공을 잃으면 단계 초기화

        # YOLO는 항상 실행되도록 신호 발행
        self.control_pub.publish(Int8(YOLO_CONTROL_RUN))

        # 상태 토픽 (항상 RUNNING)
        self.state_pub.publish(Int8(RUNNING))

    # ---------- 보조 계산 ----------
    def compute_lateral_linear_y(self, det_goal, det_ball):
        """골-공 가로 차이를 linear.y로 변환 (왼쪽 +)"""

        gap = det_goal.x_center - det_ball.x_center  # gap>0: 골이 더 오른쪽

        if abs(gap) <= GOAL_ALIGN_DEADBAND_PX:
            return 0.0, True
        # 골이 오른쪽(gap>0) → 로봇은 왼쪽(+y)으로 이동
        linear_y = LATERAL_SPEED_MAX if gap > 0 else -LATERAL_SPEED_MAX
        return linear_y, False

    def compute_yaw_angular_z(self, det_ball):
        """
        카메라 중심 대비 공의 x 오차를 angular.z로 변환 (왼쪽 회전 +, 오른쪽 회전 -)
        목표 x 좌표를 FOOT_OFFSET_PX로 조정
        """
        
        # 목표 x 좌표 설정 (카메라 중심 + 발 오프셋)
        target_x = self.cx + FOOT_OFFSET_PX
        
        # 실제 오차: 공 중심 - 목표 중심
        e_px = det_ball.x_center - target_x # 오차가 양수
        pixel_to_rad = pi / 640

        # 정렬 완료 판단
        if abs(e_px) <= YAW_ALIGN_DEADBAND_PX:
            return 0.0, True

        # 각속도 계산: 오차가 양수(공이 오른쪽)일 때 오른쪽 회전(-) 명령
        w_cmd = -e_px * pixel_to_rad 

        # 최종 클리핑
        angular_z = float(np.clip(w_cmd, -self.w_max, self.w_max))
        return angular_z, False

    # ---------- 콜백 (단계별 단독 제어) ----------
    def yolo_cb(self, msg: Yolo):
        """linear.y → angular.z → linear.x 순서의 단독 제어 FSM"""
        # 공/골대 분리 탐색
        det_ball = next((d for d in msg.detections if (d.label or "").lower() == "ball"), None)
        # print(f"det_ball.x_center: {det_ball.x_center}") # 디버그 출력 제거
        det_goal = next((d for d in msg.detections if (d.label or "").lower() == "goal"), None)
        # print(f"det_goal.x_center: {det_goal.x_center}")
        
        # Twist 초기화
        cmd = Twist()
        
        # 공/골대 둘 다 없음 → 즉시 무발행(joy로 스위칭 유도)
        if det_ball is None and det_goal is None:
            if self.twist_enabled:
                self.disable_twist_publishing()
            cmd.linear.x=0.0
            cmd.linear.y=0.0
            cmd.angular.z=0.0
            self.stage = STG_ALIGN_LATERAL
            return

        # 공 미검출(골대만 보임) → 옵션으로 즉시 중단
        if det_ball is None:
            if self.stop_immediately_if_lost and self.twist_enabled:
                self.disable_twist_publishing()
            return

        # 공을 봤으므로 타임아웃 리셋 및 퍼블리셔 복구
        self._ticks_since_seen = 0
        if not self.twist_enabled:
            self.ensure_twist_pub()

        # 상태 텍스트
        self.pi_mode.publish("goal~!")


        # 1) LATERAL 정렬 단계: linear.y만 사용 (단독)
        if self.stage == STG_ALIGN_LATERAL:
            if det_goal is None:
                # 골대를 못 보면 YAW 정렬 단계로 우회
                self.stage = STG_ALIGN_YAW
                rospy.loginfo_throttle(0.5, "[Stage] Goal not detected → switch to YAW align")
            else:
                linear_y, aligned = self.compute_lateral_linear_y(det_goal, det_ball)
                cmd.linear.y = linear_y
                # 단독 제어: 나머지는 0 (linear.x=0, angular.z=0)
                if aligned:
                    self.stage = STG_ALIGN_YAW
                    rospy.loginfo_throttle(0.5, "[Stage] LATERAL aligned → YAW align")
                self.publish_cmd(cmd)
                return  # 단독 제어 종료

        # 2) YAW 정렬 단계: angular.z만 사용 (단독, 정밀 P제어)
        if self.stage == STG_ALIGN_YAW:
            angular_z, yaw_ok = self.compute_yaw_angular_z(det_ball)
            cmd.angular.z = angular_z
            # 단독 제어: 나머지는 0 (linear.x=0, linear.y=0)
            if yaw_ok:
                self.stage = STG_DRIVE_FORWARD
                rospy.loginfo_throttle(0.5, "[Stage] YAW aligned → DRIVE forward")
            self.publish_cmd(cmd)
            return  # 단독 제어 종료

        # 3) 전진 단계: linear.x만 사용 (단독)
        if self.stage == STG_DRIVE_FORWARD:
            # 추적 FSM이 아니므로, 선속도는 하드코딩된 값을 사용
            cmd.linear.x = 0.3
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.publish_cmd(cmd)
            return

    # ---------- 발행 ----------
    def publish_cmd(self, cmd: Twist):
        if self.twist_enabled and self.twist_pub is not None:
            self.twist_pub.publish(cmd)

if __name__ == "__main__":
    BallTrack()
    rospy.spin()