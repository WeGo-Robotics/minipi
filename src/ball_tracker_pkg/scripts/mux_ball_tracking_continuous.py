#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from yolo11_detect_pkg.msg import Yolo
from math import pi

# visualize.py와 합의된 상태 코드
STATE_GO, STATE_SEARCH_LEFT, STATE_SEARCH_RIGHT, STATE_HOLD = 0, 1, 2, 4


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # Publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)  # twist_mux 사용
        self.state_pub = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.mode_pub = rospy.Publisher("/pi_mode", String, queue_size=10)

        # Subscribers
        self.yolo_topic = rospy.get_param("~detections_topic", "/YoloInfo")
        rospy.Subscriber(self.yolo_topic, Yolo, self.yolo_cb, queue_size=5)
        rospy.loginfo("Subscribed: %s", rospy.resolve_name(self.yolo_topic))

        # Image size (fallback)
        self.img_w = rospy.get_param("~image_width", 640)
        self.img_h = rospy.get_param("~image_height", 480)

        # Target filter (Ball only)
        self.ball_label = rospy.get_param("~ball_label", "ball").strip().lower()
        self.ball_class_id = rospy.get_param("~ball_class_id", -1)  # -1 => ignore
        self.min_confidence = rospy.get_param("~min_confidence", 0.3)

        # Control params
        self.tick_hz = rospy.get_param("~tick_hz", 10.0)
        self.max_angular = rospy.get_param("~max_angular", 0.6)
        self.angular_deadband = rospy.get_param("~angular_deadband", 0.02)  # rad/s
        self.angular_min = rospy.get_param("~angular_min", 0.08)  # rad/s
        self.pixel_tolerance = rospy.get_param("~pixel_tolerance_px", 50)
        self.linear_speed = rospy.get_param("~linear_speed", 0.35)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.20)
        self.alignment_hold_time = rospy.get_param("~alignment_hold_time", 0.8)
        # 기존 '정지' 기준을 '킥 드라이브 구간'으로 사용
        self.kick_drive_zone_yc = rospy.get_param("~kick_drive_zone_yc", 400)
        self.kick_linear_speed = rospy.get_param("~kick_linear_speed", 0.5)  # 구동기/마찰에 맞게 0.45~0.6 권장
        self.ball_fresh_timeout = rospy.get_param("~ball_fresh_timeout", 0.3)  # sec

        # Internal state
        self.last_ball = None  # dict(xc, yc, conf)
        self.last_ball_stamp = rospy.Time(0)
        self.is_aligned = False
        self.aligned_at = 0.0
        self.state = STATE_SEARCH_LEFT
        self.prev_ball_center = None

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self.control_tick)

    # -------------------- YOLO --------------------
    def _is_ball(self, det) -> bool:
        label_ok = False
        if hasattr(det, "label") and det.label is not None:
            label_ok = det.label.strip().lower() == self.ball_label
        class_ok = True
        if self.ball_class_id >= 0:
            class_ok = hasattr(det, "class_id") and det.class_id == self.ball_class_id
        conf_ok = True
        if hasattr(det, "conf") and det.conf is not None:
            try:
                conf_ok = float(det.conf) >= float(self.min_confidence)
            except Exception:
                conf_ok = True
        return label_ok and class_ok and conf_ok

    def yolo_cb(self, msg: Yolo):
        best = None
        best_conf = -1.0
        for det in getattr(msg, "detections", []):
            if not self._is_ball(det):
                continue
            conf = float(getattr(det, "conf", 1.0))
            if conf > best_conf:
                best_conf = conf
                best = det

        if best is not None:
            self.last_ball = {"xc": getattr(best, "x_center", self.img_w / 2.0), "yc": getattr(best, "y_center", self.img_h / 2.0), "conf": best_conf}
            self.last_ball_stamp = rospy.Time.now()
            self.prev_ball_center = (self.last_ball["xc"], self.last_ball["yc"])
        # Ball이 없으면 last_ball은 유지 (신선도는 타이머에서 판단)

    # -------------------- CONTROL --------------------
    def _fresh_ball(self) -> bool:
        if self.last_ball is None:
            return False
        age = (rospy.Time.now() - self.last_ball_stamp).to_sec()
        return age <= self.ball_fresh_timeout

    def _publish_state(self, s: int):
        self.state = s
        try:
            self.state_pub.publish(Int8(s))
        except Exception:
            pass

    def _publish_mode(self, text: str):
        try:
            self.mode_pub.publish(String(text))
        except Exception:
            pass

    def control_tick(self, _evt):
        img_w = self.img_w

        # 공이 신선하게 보이면 => 추적 "개입": /cmd_vel/auto 퍼블리시
        if self._fresh_ball():
            xc = self.last_ball["xc"]
            yc = self.last_ball["yc"]

            center_x = img_w / 2.0
            error_x = center_x - xc
            K = pi / float(img_w)  # 픽셀→라디안 선형 스케일
            w_raw = error_x * K
            # 데드밴드 & 최소회전
            if abs(w_raw) < self.angular_deadband:
                w_cmd = 0.0
            else:
                w_cmd = np.sign(w_raw) * max(abs(w_raw), self.angular_min)
            w_cmd = float(clamp(w_cmd, -self.max_angular, self.max_angular))

            twist = Twist()

            # ── 1) 킥 드라이브 구간: 멈추지 않고 강하게 밀어차기 ─────────────
            if yc > self.kick_drive_zone_yc:
                # 정렬 상태면 킥 직진, 아니면 폐루프 보정하며 전진
                if self.is_aligned and abs(error_x) <= self.pixel_tolerance:
                    twist.linear.x = self.kick_linear_speed
                    twist.angular.z = 0.0
                    self._publish_state(STATE_HOLD)
                    self._publish_mode("kick drive")
                    rospy.loginfo_throttle(0.5, "[BALL] KICK DRIVE (straight)")
                else:
                    self.is_aligned = False
                    twist.linear.x = self.kick_linear_speed * 0.8  # 약간 낮춰 보정 안정화
                    twist.angular.z = w_cmd
                    self._publish_state(STATE_GO)
                    self._publish_mode("kick drive")
                    rospy.loginfo_throttle(0.5, f"[BALL] KICK DRIVE aligning: w={w_cmd:.3f}")

            # ── 2) 일반 정렬/직진 모드 ─────────────────────────────────────
            else:
                if self.is_aligned:
                    t = rospy.get_time()
                    if t - self.aligned_at < self.alignment_hold_time:
                        twist.linear.x = self.linear_speed
                        twist.angular.z = 0.0
                        self._publish_state(STATE_HOLD)
                        self._publish_mode("hold straight")
                        rospy.loginfo_throttle(0.5, "[BALL] HOLD straight")
                    else:
                        self.is_aligned = False
                        twist.linear.x = self.min_linear_speed
                        twist.angular.z = w_cmd
                        self._publish_state(STATE_GO)
                        self._publish_mode("ball chase")
                        rospy.loginfo_throttle(0.5, f"[BALL] Resume aligning: w={w_cmd:.3f}")
                else:
                    if abs(error_x) <= self.pixel_tolerance:
                        self.is_aligned = True
                        self.aligned_at = rospy.get_time()
                        twist.linear.x = self.linear_speed
                        twist.angular.z = 0.0
                        self._publish_state(STATE_HOLD)
                        self._publish_mode("hold straight")
                        rospy.loginfo_throttle(0.5, "[BALL] Aligned -> HOLD")
                    else:
                        twist.linear.x = self.min_linear_speed
                        twist.angular.z = w_cmd
                        self._publish_state(STATE_GO)
                        self._publish_mode("ball chase")
                        rospy.loginfo_throttle(0.5, f"[BALL] Aligning: ex={error_x:.1f}px w={w_cmd:.3f}")

            # 공이 보이는 동안에만 퍼블리시 → 벽 회피와 토픽 충돌 최소화
            try:
                self.cmd_pub.publish(twist)
            except Exception:
                pass
            return

        # 공이 신선하지 않으면 퍼블리시 안 함 (벽 회피 등에 제어권 반환)
        if self.prev_ball_center is not None:
            cx_prev = self.prev_ball_center[0]
            if cx_prev < self.img_w / 2.0:
                self._publish_state(STATE_SEARCH_LEFT)
            else:
                self._publish_state(STATE_SEARCH_RIGHT)
            self._publish_mode("finding ball")
        else:
            self._publish_state(STATE_SEARCH_LEFT)
            self._publish_mode("finding ball")

        self.is_aligned = False


if __name__ == "__main__":
    BallTrack()
    rospy.spin()
