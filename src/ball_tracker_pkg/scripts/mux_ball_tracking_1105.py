#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# llm_conv.py에서 물건 검출하는 코드와 같은 정렬 방식 사용
# + visualize.py 호환: /yolo_state(Int8), /pi_mode(String) 퍼블리시

import rospy
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from std_msgs.msg import Int8, String
from math import pi


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# visualize.py의 state_map과 맞춤
STATE_GO = 0  # 추종/직진
STATE_SEARCH_LEFT = 1
STATE_SEARCH_RIGHT = 2
STATE_HOLD = 4  # 정렬 후 직진 유지 구간


class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # Publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.pi_mode_pub = rospy.Publisher("/pi_mode", String, queue_size=10)
        self.cmd_vel_msg = Twist()

        # Subscribers
        self.yolo_sub = rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)

        # Image size (W, H)
        self.img_size = (
            rospy.get_param("~image_width", 640),
            rospy.get_param("~image_height", 480),
        )

        # 추적 타겟 정의 (라벨/클래스/신뢰도)
        self.ball_label = rospy.get_param("~ball_label", "ball").strip().lower()
        self.ball_class_id = rospy.get_param("~ball_class_id", -1)  # -1이면 사용 안함
        self.min_conf = rospy.get_param("~min_confidence", 0.30)

        # State & history
        self.prev_ball = None  # [x_center, y_center]
        self.last_detection = None  # {"x_center":..., "y_center":...}
        self.frame_cnt = 0
        self.state = STATE_SEARCH_LEFT  # 초기엔 좌측 탐색
        self._last_state_pub = None

        # Search / blind-forward parameters
        self.center_margin = rospy.get_param("~center_margin_px", 40)
        self.blind_forward_ticks = rospy.get_param("~blind_forward_ticks", 5)
        self._blind_cnt = 0

        # 중앙 정렬 + 직진 유지 파라미터 (객체 추종 코드 방식 준수)
        self.LINEAR_SPEED = rospy.get_param("~linear_speed", 0.35)
        self.MIN_LINEAR_SPEED = rospy.get_param("~min_linear_speed", 0.2)
        self.PIXEL_TOLERANCE = rospy.get_param("~pixel_tolerance_px", 50)
        self.FINAL_STOP_Y_CENTER = rospy.get_param("~final_stop_y_center", 400)
        self.ALIGNMENT_HOLD_TIME = rospy.get_param("~alignment_hold_time", 0.8)
        self.MAX_ANGULAR = rospy.get_param("~max_angular", 0.6)

        # 정렬 상태
        self.is_aligned = False
        self.alignment_time = None

        # 10Hz control loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

    # -------------------- 메인 제어 루프 --------------------
    def tracking(self, event):
        desired_state = self.state  # 기본 유지

        if self.last_detection is not None:
            xc = self.last_detection["x_center"]
            yc = self.last_detection["y_center"]
            img_w = self.img_size[0]
            center_x = img_w / 2.0
            error_x = center_x - xc

            # 각속도 계산 (물체 추종 코드와 동일)
            calculated_angular_z = error_x * (pi / img_w)
            should_stop = False

            # 1) 충분히 가까우면 정지
            if yc > self.FINAL_STOP_Y_CENTER:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                should_stop = True
                self.is_aligned = False
                desired_state = STATE_GO
                self._publish_pi_mode("near target")
                rospy.loginfo("[MOVE] reached near target. stop.")

            # 2) 정렬 상태면 일정 시간 직진 유지(각속도 0)
            elif self.is_aligned:
                now = rospy.get_time()
                if now - (self.alignment_time or now) < self.ALIGNMENT_HOLD_TIME:
                    self.cmd_vel_msg.linear.x = self.LINEAR_SPEED
                    self.cmd_vel_msg.angular.z = 0.0
                    desired_state = STATE_HOLD
                    self._publish_pi_mode("hold straight")
                    rospy.loginfo(f"[MOVE] hold straight ({now - self.alignment_time:.2f}s).")
                else:
                    self.is_aligned = False
                    desired_state = STATE_GO
                    self._publish_pi_mode("ball chase")
                    rospy.loginfo("[MOVE] hold done. back to normal.")

            # 3) 중앙 오차 허용 범위 → 정렬 진입 & 직진 유지 시작
            elif abs(error_x) < self.PIXEL_TOLERANCE:
                self.is_aligned = True
                self.alignment_time = rospy.get_time()
                self.cmd_vel_msg.linear.x = self.LINEAR_SPEED
                self.cmd_vel_msg.angular.z = 0.0
                desired_state = STATE_HOLD
                self._publish_pi_mode("hold straight")
                rospy.loginfo("[MOVE] aligned -> go straight & hold.")

            # 4) 정렬 중: 느리게 전진 + 각속도
            else:
                self.cmd_vel_msg.linear.x = self.MIN_LINEAR_SPEED
                self.cmd_vel_msg.angular.z = clamp(calculated_angular_z, -self.MAX_ANGULAR, self.MAX_ANGULAR)
                desired_state = STATE_GO
                self._publish_pi_mode("ball chase")
                rospy.loginfo(f"[MOVE] aligning L:{self.cmd_vel_msg.linear.x:.2f} A:{self.cmd_vel_msg.angular.z:.2f}")

            # 퍼블리시
            self._publish_state(desired_state)
            self.cmd_pub.publish(self.cmd_vel_msg)

            # 근접 정지 처리 후 상태 리셋
            if should_stop:
                self.last_detection = None
                self.prev_ball = None
                self.frame_cnt = 0
                self._blind_cnt = 0
            return

        # ---- 최근 탐지가 없을 때: 블라인드 전진 → 탐색 회전 ----
        self.frame_cnt += 1
        if self.frame_cnt > 10:
            img_center_x = self.img_size[0] // 2
            if self.prev_ball is not None:
                # 마지막 위치가 중심 근처면 잠깐 블라인드 전진
                if abs(self.prev_ball[0] - img_center_x) <= self.center_margin and self._blind_cnt < self.blind_forward_ticks:
                    self.cmd_vel_msg.linear.x = 0.1
                    self.cmd_vel_msg.angular.z = 0.0
                    desired_state = STATE_GO
                    self._blind_cnt += 1
                    self._publish_pi_mode("ball chase")
                    rospy.loginfo(f"[SEARCH] blind forward {self._blind_cnt}/{self.blind_forward_ticks}")
                else:
                    # 좌/우로 사라진 경우 해당 방향으로 탐색
                    self.cmd_vel_msg.linear.x = 0.0
                    if self.prev_ball[0] < img_center_x:
                        self.cmd_vel_msg.angular.z = 1.0
                        desired_state = STATE_SEARCH_LEFT
                        self._publish_pi_mode("finding ball")
                        rospy.loginfo("[SEARCH] rotate left")
                    else:
                        self.cmd_vel_msg.angular.z = -1.0
                        desired_state = STATE_SEARCH_RIGHT
                        self._publish_pi_mode("finding ball")
                        rospy.loginfo("[SEARCH] rotate right")
            else:
                # 히스토리 없으면 기본 왼쪽 탐색
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 1.0
                desired_state = STATE_SEARCH_LEFT
                self._publish_pi_mode("finding ball")
                rospy.loginfo("[SEARCH] rotate left (no history)")

        # 상태/명령 퍼블리시
        self._publish_state(desired_state)
        self.cmd_pub.publish(self.cmd_vel_msg)

    # -------------------- YOLO 콜백 --------------------
    def _is_ball_detection(self, det) -> bool:
        """라벨/클래스/신뢰도 기준으로 'Ball'만 유효 처리"""
        # 라벨
        label_ok = False
        if hasattr(det, "label") and det.label is not None:
            label_ok = det.label.strip().lower() == self.ball_label
        # 클래스 (옵션)
        class_ok = True
        if self.ball_class_id >= 0:
            class_ok = hasattr(det, "class_id") and det.class_id == self.ball_class_id
        # 신뢰도 (있으면 사용)
        conf_ok = True
        if hasattr(det, "conf") and det.conf is not None:
            try:
                conf_ok = float(det.conf) >= float(self.min_conf)
            except Exception:
                conf_ok = True
        return label_ok and class_ok and conf_ok

    def yolo_cb(self, msg: Yolo):
        """오직 'Ball'만 후보로 채택, 그 중 최고 신뢰도 1개만 사용"""
        best = None
        best_conf = -1.0

        if getattr(msg, "count", 0):
            for det in msg.detections:
                if not self._is_ball_detection(det):
                    continue
                conf = float(getattr(det, "conf", 1.0))
                if conf > best_conf:
                    best_conf = conf
                    best = det

        if best is not None:
            self.last_detection = {"x_center": best.x_center, "y_center": best.y_center}
            self.prev_ball = [best.x_center, best.y_center]
            self.frame_cnt = 0
            self._blind_cnt = 0
            # 상태는 추적/정렬 로직에서 결정
            rospy.loginfo(f"[YOLO] Ball detected (conf={best_conf:.2f}).")
        else:
            self.last_detection = None  # 다른 물체만 있던 프레임도 추적 갱신 금지

    # -------------------- 퍼블리시 유틸 --------------------
    def _publish_state(self, s: int):
        self.state = s
        # 항상 퍼블리시(시각화 신선도 유지)
        try:
            self.state_pub.publish(Int8(self.state))
        except Exception:
            pass

    def _publish_pi_mode(self, text: str):
        try:
            self.pi_mode_pub.publish(String(text))
        except Exception:
            pass


if __name__ == "__main__":
    BallTrack()
    rospy.spin()
