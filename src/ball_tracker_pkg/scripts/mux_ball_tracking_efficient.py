#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from yolo11_detect_pkg.msg import Yolo
from math import pi

# 상태 상수
GO, HOLD = 0, 4

# YOLO 실행/정지 플래그 값
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0

# pixel to distance 인자 (y기준)
RATIONAL_COEF = (-84.166458, 50689.309424, 25.534673, 1)

def get_attr(obj, names, default=None):
    """여러 후보 속성명 중 먼저 존재하는 값을 반환"""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return default

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # 퍼블리셔(항상 유지: rqt에서 보이도록)
        self.twist_pub   = rospy.Publisher("/cmd_vel/auto", Twist, queue_size=10)
        self.state_pub   = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.control_pub = rospy.Publisher("/yolo_run_control", Int8, queue_size=1)
        self.pi_mode     = rospy.Publisher("/pi_mode", String, queue_size=10)
        rospy.loginfo("Publishers ready: /cmd_vel/auto, /yolo_state, /yolo_run_control")

        # 입력 디텍션 토픽
        self.detections_topic = rospy.get_param("~detections_topic", "/YoloInfo")
        rospy.Subscriber(self.detections_topic, Yolo, self.yolo_cb)
        rospy.loginfo("Subscribed detections: %s", rospy.resolve_name(self.detections_topic))

        # 이미지 크기(필요시 파라미터로 조정)
        self.img_w = rospy.get_param("~img_w", 640)
        self.img_h = rospy.get_param("~img_h", 480)

        # 타이밍/동작 파라미터
        self.tick_hz     = rospy.get_param("~tick_hz", 10)
        self.rush_ticks  = rospy.get_param("~rush_ticks", 30)          # GO 유지(틱)
        self.arm_ticks   = rospy.get_param("~arm_ticks", 10)           # 출발 전 대기(틱)
        self.lost_limit  = rospy.get_param("~lost_limit_ticks", 30)    # 미검출 연속(틱)

        # 선속도 파라미터
        self.v_min = rospy.get_param("~v_min", 0.1)
        self.v_max = rospy.get_param("~v_max", 0.4)

        # 각속도 파라미터
        self.w_max      = rospy.get_param("~w_max", 0.2)
        self.w_deadband = rospy.get_param("~w_deadband", 0.1)  # (미사용 중, 후처리 시 활용 가능)
        self.w_gamma    = rospy.get_param("~w_gamma", 5.0)     # (미사용 중, 비선형 맵핑 시 활용)
        self.w_dir      = rospy.get_param("~w_dir", -1.0)      # (미사용 중, 필요 시 부호 반전용)
        self.w_gain     = rospy.get_param("~w_gain", 1.0)      # (미사용 중, 필요 시 스케일 조정)
        self.w_min      = rospy.get_param("~w_min", 0.0)       # (미사용 중, 최소 회전량)

        # ▶ 공 라벨/신뢰도 조건 (여기가 핵심!)
        self.ball_labels = [s.lower() for s in rospy.get_param("~ball_labels", ["ball"])]
        self.min_conf    = float(rospy.get_param("~min_conf", 0.25))  # 0~1 범위 추천

        # 내부 메시지 버퍼/상태
        self.twist_msg = Twist()
        self.state             = HOLD
        self._go_left          = 0
        self._arm_left         = 0
        self._pending_vw       = None
        self._arm_timer        = None
        self._ticks_since_seen = self.lost_limit + 1
        self.twist_enabled     = True  # 퍼블리셔 유지, 발행만 on/off

        # 주기 타이머
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self.loop)

    @staticmethod
    def pixel_to_distance(y_pix, method):
        if len(method) == 2:
            return method[0] * y_pix + method[1]
        else:
            return (method[0] * y_pix + method[1]) / (method[2] * y_pix + method[3])

    def loop(self, _):
        rospy.loginfo_throttle(
            0.5,
            f"STATE={self.state} GO_LEFT={self._go_left} seen_ticks={self._ticks_since_seen} arm_left={self._arm_left}"
        )

        # HOLD일 때만 '미검출 경과' 카운트
        if self.state == HOLD:
            self._ticks_since_seen += 1
            # 아밍 중엔 무발행 트리거 보류(경합 방지)
            if self._arm_left == 0 and self._ticks_since_seen > self.lost_limit and self.twist_enabled:
                self.disable_twist_publishing()

        if self.state == HOLD:
            # HOLD에서는 /cmd_vel 발행하지 않음(0 Twist도 금지)
            pass

        elif self.state == GO:
            if self._go_left > 0:
                self._go_left -= 1
                if self.twist_enabled:
                    self.twist_pub.publish(self.twist_msg)
                    rospy.loginfo_throttle(1.0, "Publishing /cmd_vel/auto ...")
            else:
                self._enter_hold()

        # YOLO 실행/정지 플래그
        yolo_control_value = YOLO_CONTROL_STOP if self.state == GO else YOLO_CONTROL_RUN
        self.control_pub.publish(Int8(yolo_control_value))
        self.state_pub.publish(Int8(self.state))

    # 발행 제어
    def disable_twist_publishing(self):
        self.twist_enabled = False
        if self.state != HOLD:
            self._enter_hold()
        rospy.logwarn("CMD publishing disabled -> /cmd_vel/auto는 보이지만 실제 발행은 중지.")

    def enable_twist_publishing(self):
        self.twist_enabled = True
        rospy.loginfo("CMD publishing enabled.")

    # 상태 전이
    def _enter_hold(self):
        self.state = HOLD
        self._go_left = 0
        # 다음 사이클을 위해 아밍 관련 플래그 리셋
        self._arm_left = 0
        self._pending_vw = None
        if self._arm_timer is not None:
            try:
                self._arm_timer.shutdown()
            except Exception:
                pass
            self._arm_timer = None

    def _enter_go_with(self, v, w):
        if not self.twist_enabled:
            self.enable_twist_publishing()
        self.state = GO
        self._go_left = self.rush_ticks
        self.twist_msg.linear.x  = float(np.clip(v, self.v_min, self.v_max))
        self.twist_msg.angular.z = float(np.clip(w, -self.w_max, self.w_max))

    # -------------------------
    # 공 디텍션 필터 유틸
    # -------------------------
    def _is_ball_detection(self, det):
        label = (get_attr(det, ["label", "class_name", "name", "cls_name"], "") or "").strip().lower()
        if label not in self.ball_labels:
            return False
        # confidence 필드 대응: confidence, conf, score, prob, probability
        conf = get_attr(det, ["confidence", "conf", "score", "prob", "probability"], None)
        if conf is None:
            # 신뢰도 값이 없으면 보수적으로 불허(원하면 허용 가능)
            return False
        try:
            conf = float(conf)
        except Exception:
            return False
        return conf >= self.min_conf

    # YOLO 콜백
    def yolo_cb(self, msg: Yolo):
        # 콜백 들어오면 '최근 본 것'으로 간주하여 리셋
        self._ticks_since_seen = 0

        # GO 중에는 무시
        if self.state != HOLD:
            self.pi_mode.publish("Chasing ball...")
            return

        detections = getattr(msg, "detections", [])
        if not detections:
            return

        # ── 여기서 공 라벨 + 신뢰도 통과하는 디텍션만 사용 ──
        ball_dets = [d for d in detections if self._is_ball_detection(d)]
        if not ball_dets:
            # 공이 아니면 절대 움직이지 않음
            self.pi_mode.publish("no ball")
            return

        # 여러 개면 가장 중앙에 가까운 공 선택(선택 로직 필요시 교체)
        cx_frame = self.img_w * 0.5
        def det_center_x(d):
            return get_attr(d, ["x_center", "xc", "cx"], cx_frame)
        det_ball = min(ball_dets, key=lambda d: abs(det_center_x(d) - cx_frame))
        self.pi_mode.publish("ball detect")

        # 이미 아밍 중이면 중복 무장 방지
        if self._arm_left > 0 or self._pending_vw is not None:
            return

        # --- v 계산 (거리 기반) ---
        y_center   = float(get_attr(det_ball, ["y_center", "yc", "cy"], self.img_h * 0.7))
        distance_m = self.pixel_to_distance(y_center, RATIONAL_COEF)
        v          = float(np.clip(distance_m, self.v_min, self.v_max))

        # --- w 계산 (px -> rad 변환) ---
        x_center = float(get_attr(det_ball, ["x_center", "xc", "cx"], cx_frame))
        diff_e   = (self.img_w * 0.5) - x_center  # 왼쪽(픽셀)<0, 오른쪽>0 → 부호는 아래 변환에서 처리
        pixel_to_rad = pi / float(self.img_w)     # 단순 FOV≈π rad 가정
        w_cmd = diff_e * pixel_to_rad

        # 필요시 부호/스케일 조정: (현재는 직진 매핑, deadband/gain 활용하려면 아래 주석 해제)
        # if abs(w_cmd) < self.w_deadband: w_cmd = 0.0
        # w_cmd = np.sign(w_cmd) * max(self.w_min, min(abs(w_cmd) * self.w_gain, self.w_max))
        w = float(np.clip(w_cmd, -self.w_max, self.w_max))

        rospy.loginfo(f"ARMING: v={v:.3f}, w={w:.3f}")
        self._pending_vw = (v, w)
        self._arm_left   = self.arm_ticks
        self._start_arming_if_needed()

    def _start_arming_if_needed(self):
        if self.state != HOLD or self._arm_left <= 0:
            return
        if self._arm_timer is None:
            self._arm_timer = rospy.Timer(
                rospy.Duration(1.0 / self.tick_hz),
                self._arming_tick,
                oneshot=False
            )

    def _arming_tick(self, _):
        if self.state != HOLD:
            if self._arm_timer is not None:
                try:
                    self._arm_timer.shutdown()
                except Exception:
                    pass
                self._arm_timer = None
            return

        if self._arm_left > 0:
            self._arm_left -= 1

        if self._arm_left == 0:
            if self._arm_timer is not None:
                try:
                    self._arm_timer.shutdown()
                except Exception:
                    pass
                self._arm_timer = None

            if self._pending_vw is not None:
                v, w = self._pending_vw
                self._pending_vw = None
                rospy.loginfo(f"ENTER GO: v={v:.3f}, w={w:.3f}, rush_ticks={self.rush_ticks}")
                self._enter_go_with(v, w)

if __name__ == "__main__":
    BallTrack()
    rospy.spin()
