#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from yolo11_detect_pkg.msg import Yolo
from math import pi

# 상태 상수 (GO 상태는 이제 tracking 플래그로 대체)
HOLD = 0

# YOLO 실행/정지 플래그 값
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0

# pixel to distance 인자 (y기준) - 미사용되지만 원래 코드 유지를 위해 남겨둠
RATIONAL_COEF = (-84.166458, 50689.309424, 25.534673, 1)

# 근접 정지 y 픽셀 (RkllamaPi와 유사한 명확한 정지 조건 추가)
FINAL_STOP_Y_CENTER = 400


class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # 퍼블리셔
        self.twist_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.control_pub = rospy.Publisher("/yolo_run_control", Int8, queue_size=1)
        self.pi_mode = rospy.Publisher("/pi_mode", String, queue_size=10)
        rospy.loginfo("Publishers ready: /cmd_vel/auto, /yolo_state, /yolo_run_control")

        # 입력 디텍션 토픽
        self.detections_topic = rospy.get_param("~detections_topic", "/YoloInfo")
        rospy.Subscriber(self.detections_topic, Yolo, self.yolo_cb)
        rospy.loginfo("Subscribed detections: %s", rospy.resolve_name(self.detections_topic))

        # 내부 메시지 버퍼
        self.twist_msg = Twist()

        # 이미지 크기
        self.img_w, self.img_h = 640, 480

        # 타이밍/동작 파라미터
        self.tick_hz = rospy.get_param("~tick_hz", 10)
        # self.rush_ticks = rospy.get_param("~rush_ticks", 30)  # 제거
        self.lost_limit = rospy.get_param("~lost_limit_ticks", 30)  # 미검출 연속(틱)

        # 선속도 파라미터 (v_min, v_max 대신 일반 LINEAR_SPEED 사용)
        self.v_min = rospy.get_param("~v_min", 0.1)
        self.v_max = rospy.get_param("~v_max", 0.4)
        self.linear_speed = rospy.get_param("~linear_speed", 0.35)  # 새로운 기본 속도

        # 각속도 파라미터 (P-제어에 사용)
        self.w_max = rospy.get_param("~w_max", 0.6)  # 클리핑 최대값 상향 (RkllamaPi 유사)
        self.w_deadband = rospy.get_param("~w_deadband", 10.0)  # 픽셀 오차 데드밴드 (픽셀 단위로 해석)
        # self.w_gamma = rospy.get_param("~w_gamma", 5.0) # 미사용
        self.w_dir = rospy.get_param("~w_dir", -1.0)  # 회전 방향 보정
        self.w_gain = rospy.get_param("~w_gain", 0.005)  # 핵심 P-Gain
        self.w_min = rospy.get_param("~w_min", 0.1)  # 최소 회전 속도 (deadband 회피)

        # 내부 상태
        self.state = HOLD
        self.is_tracking = False  # 공을 추적 중인지 여부 (GO 대체)
        self._ticks_since_seen = self.lost_limit + 1
        self.twist_enabled = True

        # 최신 YOLO 정보를 저장할 구조체
        self.latest_detection_info = None  # (x_center, y_center)

        # 주기 타이머 (실시간 제어)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self.loop)

    @staticmethod
    def pixel_to_distance(y_pix, method):
        if len(method) == 2:
            return method[0] * y_pix + method[1]
        else:
            return (method[0] * y_pix + method[1]) / (method[2] * y_pix + method[3])

    def loop(self, _):
        """
        주기 제어 루프 (10Hz)
        is_tracking = True 일 때 실시간으로 cmd_vel 발행
        """
        rospy.loginfo_throttle(0.5, f"STATE={self.state} is_tracking={self.is_tracking} seen_ticks={self._ticks_since_seen}")

        # HOLD일 때만 '미검출 경과' 카운트
        if self.state == HOLD:
            self._ticks_since_seen += 1
            if self._ticks_since_seen > self.lost_limit and self.twist_enabled:
                self.disable_twist_publishing()

        # ── 실시간 추적 로직 ────────────────────────────────────────────────
        if self.is_tracking and self.twist_enabled:
            self._process_tracking()

        elif not self.is_tracking:
            # 추적 중이 아니면 Twist 0 발행
            # self.twist_pub.publish(Twist())
            pass

        # YOLO 실행/정지 플래그
        # HOLD 상태일 때만 YOLO 실행, 추적 중에는 멈춤 (원래 로직 유지)
        yolo_control_value = YOLO_CONTROL_STOP if self.is_tracking else YOLO_CONTROL_RUN
        self.control_pub.publish(Int8(yolo_control_value))
        self.state_pub.publish(Int8(self.state))

    def _process_tracking(self):
        """
        최신 YOLO 정보(self.latest_detection_info)를 사용하여 v, w를 계산하고 발행
        """
        if self.latest_detection_info is None:
            rospy.logwarn_throttle(1.0, "[TRACK] Lost target or no recent data. Stopping.")
            self._stop_tracking()
            return

        x_center, y_center = self.latest_detection_info
        cx = self.img_w * 0.5
        error_x = cx - x_center  # 화면 중앙(cx) - 공 중앙(x_center)

        twist = Twist()
        should_stop = False

        # 1. 근접 정지 조건
        if y_center > FINAL_STOP_Y_CENTER:
            should_stop = True
            rospy.loginfo("[TRACK] Reached near target (y_center > 400). Stop tracking.")

        # 2. 각속도 (P-제어) 계산
        if not should_stop:
            # P-제어: error * Gain * Direction
            w_cmd = error_x * self.w_gain * self.w_dir

            # 데드밴드 적용 (w_deadband를 픽셀 오차로 해석)
            if abs(error_x) < self.w_deadband:
                w = 0.0
            else:
                # 최소 회전 속도 적용 (deadband를 벗어났을 경우)
                sign = np.sign(w_cmd)
                w_abs = max(abs(w_cmd), self.w_min)
                w = sign * w_abs

            # 최대 각속도 클리핑
            twist.angular.z = float(np.clip(w, -self.w_max, self.w_max))

            # 3. 선속도 계산 (고정 속도 혹은 거리 기반)
            # 현재는 고정 속도 유지 (RkllamaPi 방식)
            twist.linear.x = float(np.clip(self.linear_speed, self.v_min, self.v_max))

            rospy.loginfo_throttle(0.2, f"[TRACK] L:{twist.linear.x:.2f} A:{twist.angular.z:.2f} Err:{error_x:.1f}")

        self.twist_pub.publish(twist)

        if should_stop:
            self._stop_tracking()
            self.pi_mode.publish("Arrived")

    def _stop_tracking(self):
        """추적 상태 종료 및 리셋"""
        self.is_tracking = False
        self.state = HOLD
        self.latest_detection_info = None
        self.twist_pub.publish(Twist())  # 정지 명령 발행
        self.pi_mode.publish("HOLD")

    # 발행 제어
    def disable_twist_publishing(self):
        self.twist_enabled = False
        self.is_tracking = False
        self.state = HOLD
        rospy.logwarn("CMD publishing disabled -> /cmd_vel/auto는 보이지만 실제 발행은 중지.")

    def enable_twist_publishing(self):
        self.twist_enabled = True
        rospy.loginfo("CMD publishing enabled.")

    # 상태 전이 (이제는 HOLD만 존재)
    def _enter_hold(self):
        self._stop_tracking()

    # YOLO 콜백: 여기서 실질적인 제어를 수행하지 않고, 최신 정보만 업데이트
    def yolo_cb(self, msg: Yolo):
        self._ticks_since_seen = 0

        detections = getattr(msg, "detections", [])
        if not detections:
            # YOLO 디텍션이 없으면 최신 정보 무효화
            self.latest_detection_info = None
            return

        # 'ball' 우선, 없으면 첫 디텍션
        det_ball = next((d for d in detections if (getattr(d, "label", "") or "").lower() == "ball"), None)
        if det_ball is None:
            det_ball = detections[0]

        # 최신 위치 정보 저장
        self.latest_detection_info = (getattr(det_ball, "x_center"), getattr(det_ball, "y_center"))

        # HOLD 상태일 때만 추적 시작
        if self.state == HOLD:
            self.is_tracking = True
            self.state = 1  # 임의의 추적 상태 값 (GO 대체)
            self.enable_twist_publishing()
            rospy.loginfo("Tracking started from HOLD.")
            self.pi_mode.publish("Chasing ball...")


if __name__ == "__main__":
    try:
        BallTrack()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
