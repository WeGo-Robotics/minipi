#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from yolo11_detect_pkg.msg import Yolo

# 상태 코드 (visualizer랑 맞춰 써야 함)
STATE_GO, STATE_SEARCH_LEFT, STATE_SEARCH_RIGHT, STATE_HOLD = 0, 1, 2, 4


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PersonTrack:
    def __init__(self):
        rospy.init_node("person_tracker", anonymous=False)

        # ---------------- 파라미터 ----------------
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel/auto")
        self.detections_topic = rospy.get_param("~detections_topic", "/yolo/detections")
        self.state_topic = rospy.get_param("~state_topic", "/ball/state")  # 토픽 이름은 필요하면 나중에 /person/state 로 변경
        self.mode_topic = rospy.get_param("~mode_topic", "/pi_mode")

        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 480)
        self.img_cx = self.image_width / 2.0
        self.img_cy = self.image_height / 2.0

        # 🔥 타겟: 사람(person)
        self.person_label = rospy.get_param("~person_label", "person").strip().lower()
        self.person_class_id = rospy.get_param("~person_class_id", -1)  # -1이면 class_id는 무시
        self.min_confidence = rospy.get_param("~min_confidence", 0.3)

        # 제어 게인
        self.kp_angular = rospy.get_param("~kp_angular", 0.0025)
        self.kp_linear = rospy.get_param("~kp_linear", 0.0015)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1.0)
        self.max_lin_vel = rospy.get_param("~max_lin_vel", 0.4)

        # 사람 안 보일 때 회전 속도
        self.search_ang_vel = rospy.get_param("~search_ang_vel", 0.8)

        # bbox 면적 기반 거리 제어
        self.target_area = rospy.get_param("~target_area", 0.10)  # normalized
        self.area_tolerance = rospy.get_param("~area_tolerance", 0.04)

        # 히스토리 / 타이밍
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.7)
        self.history_length = rospy.get_param("~history_length", 10)
        self.max_no_detection = rospy.get_param("~max_no_detection", 0.8)
        self.align_timeout = rospy.get_param("~align_timeout", 1.0)
        self.max_dist_lost = rospy.get_param("~max_dist_lost", 80.0)

        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        # ---------------- 상태 변수 ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, Int8, queue_size=10)
        self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=10)

        self.last_person = None  # 최근 검출 정보 (dict)
        self.last_person_stamp = None  # 최근 검출 시각
        self.prev_person_center = None  # 마지막 위치 (x,y)
        self.no_det_elapsed = 0.0  # 마지막 검출 이후 경과 시간

        self.is_aligned = False  # 중심 정렬 상태
        self.align_start_time = None

        self.last_cmd_time = rospy.Time.now()

        # YOLO detections 구독
        rospy.Subscriber(self.detections_topic, Yolo, self.yolo_callback, queue_size=1)

        rospy.loginfo("PersonTrack node started (person tracking)")
        rospy.loginfo("  detections_topic: %s", self.detections_topic)
        rospy.loginfo("  cmd_topic:        %s", self.cmd_topic)
        rospy.loginfo("  image size:       %dx%d", self.image_width, self.image_height)
        rospy.loginfo("  target label:     %s", self.person_label)

        # 시작 상태: 사람 찾는 중
        self._publish_state(STATE_SEARCH_LEFT)
        self._publish_mode("finding person")

        self.rate = rospy.Rate(60)
        self.main_loop()

    # ----------------------------------------------------------------------
    # 헬퍼 함수들
    # ----------------------------------------------------------------------
    def _publish_state(self, state: int):
        msg = Int8()
        msg.data = int(state)
        self.state_pub.publish(msg)

    def _publish_mode(self, text: str):
        msg = String()
        msg.data = text
        self.mode_pub.publish(msg)

    def _publish_cmd(self, lin_x: float, ang_z: float):
        self.last_cmd_time = rospy.Time.now()
        cmd = Twist()
        cmd.linear.x = lin_x
        cmd.angular.z = ang_z
        self.cmd_pub.publish(cmd)

    def _stop_cmd_if_timeout(self):
        # 일정 시간 동안 명령 안 보냈으면 정지 cmd
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            cmd = Twist()
            self.cmd_pub.publish(cmd)

    def _is_person(self, det) -> bool:
        """
        YOLO detection 이 우리가 원하는 '사람(person)'인지 판단
        """
        label = (det.label or "").strip().lower()
        label_ok = label == self.person_label

        class_ok = True
        if self.person_class_id >= 0:
            class_ok = det.class_id == self.person_class_id

        conf = float(getattr(det, "conf", 1.0))
        if conf < self.min_confidence:
            return False

        return label_ok and class_ok

    # ----------------------------------------------------------------------
    # YOLO 콜백
    # ----------------------------------------------------------------------
    def yolo_callback(self, msg: Yolo):
        if not hasattr(msg, "detections") or len(msg.detections) == 0:
            # 검출 없음
            self.last_person = None
            if self.last_person_stamp is not None:
                dt = (rospy.Time.now() - self.last_person_stamp).to_sec()
                self.no_det_elapsed = dt
            else:
                self.no_det_elapsed = 999.0
            return

        best_det = None
        best_conf = -1.0

        for det in msg.detections:
            if not self._is_person(det):
                continue
            conf = float(getattr(det, "conf", 1.0))
            if conf > best_conf:
                best_conf = conf
                best_det = det

        if best_det is not None:
            xc = float(getattr(best_det, "x_center", self.img_cx))
            yc = float(getattr(best_det, "y_center", self.img_cy))
            w = float(getattr(best_det, "w", 0.0))
            h = float(getattr(best_det, "h", 0.0))

            self.last_person = {
                "xc": xc,
                "yc": yc,
                "w": w,
                "h": h,
                "conf": best_conf,
            }
            self.last_person_stamp = rospy.Time.now()
            self.prev_person_center = (xc, yc)
            self.no_det_elapsed = 0.0
        else:
            self.last_person = None
            if self.last_person_stamp is not None:
                dt = (rospy.Time.now() - self.last_person_stamp).to_sec()
                self.no_det_elapsed = dt
            else:
                self.no_det_elapsed = 999.0

    # ----------------------------------------------------------------------
    # 메인 루프
    # ----------------------------------------------------------------------
    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                self._step()
            except Exception as e:
                rospy.logerr_throttle(1.0, "Exception in main_loop: %s", e)
            self.rate.sleep()

        # 종료 시 정지
        self._publish_cmd(0.0, 0.0)

    # def _step(self):
    #     self._stop_cmd_if_timeout()

    #     now = rospy.Time.now()

    #     # 사람을 오래 못 봤으면 검색 모드
    #     if self.last_person is None or self.no_det_elapsed > self.max_no_detection:
    #         self._search_mode()
    #         return

    #     # ---------------- 사람 추적 로직 ----------------
    #     xc = float(self.last_person["xc"])
    #     yc = float(self.last_person["yc"])

    #     dx = xc - self.img_cx
    #     dy = self.img_cy - yc  # 화면 위쪽이 + 가 되도록

    #     # 화면 기준 정규화 (대략 [-1, 1] 범위)
    #     nx = dx / (self.image_width * 0.5)
    #     ny = dy / (self.image_height * 0.5)

    #     # 회전 제어: 사람을 화면 중앙으로 오게
    #     ang_z = -self.kp_angular * nx  # 오른쪽 있으면 음, 왼쪽 있으면 양
    #     ang_z = clamp(ang_z, -self.max_ang_vel, self.max_ang_vel)

    #     # 거리 제어: bbox 면적 기반
    #     w = float(self.last_person["w"])
    #     h = float(self.last_person["h"])
    #     area = (w * h) / float(self.image_width * self.image_height + 1e-6)

    #     err_area = self.target_area - area
    #     lin_x = self.kp_linear * err_area
    #     lin_x = clamp(lin_x, -self.max_lin_vel, self.max_lin_vel)

    #     # 일정 범위 안에 들어오면 전진 멈춤
    #     if abs(err_area) < self.area_tolerance:
    #         lin_x = 0.0

    #     # 중심 정렬 여부 체크
    #     if abs(nx) < 0.05:  # x 오차가 작으면 정렬 상태
    #         if not self.is_aligned:
    #             self.is_aligned = True
    #             self.align_start_time = now
    #         else:
    #             if (now - self.align_start_time).to_sec() > self.align_timeout:
    #                 self._publish_state(STATE_HOLD)
    #                 self._publish_mode("person chase")
    #     else:
    #         self.is_aligned = False

    #     # 실제 명령 퍼블리시
    #     self._publish_cmd(lin_x, ang_z)
    #     self._publish_state(STATE_GO)
    #     self._publish_mode("person chase")

    def _step(self):
        self._stop_cmd_if_timeout()
        now = rospy.Time.now()

        # ------------------------------------------------
        # 1. 사람을 잃어버린 경우 → 탐색
        # ------------------------------------------------
        if self.last_person is None or self.no_det_elapsed > self.max_no_detection:
            self._search_mode()
            return

        # ------------------------------------------------
        # 2. 위치 정보 계산
        # ------------------------------------------------
        xc = float(self.last_person["xc"])
        dx = xc - self.img_cx
        nx = dx / (self.image_width * 0.5)

        w = float(self.last_person["w"])
        h = float(self.last_person["h"])
        area = (w * h) / float(self.image_width * self.image_height)

        err_area = self.target_area - area

        # ------------------------------------------------
        # 3. 상태 판단
        # ------------------------------------------------
        is_close = area > self.area_tolerance
        aligned = abs(nx) < 0.05

        # ------------------------------------------------
        # 4. 기본 출력값
        # ------------------------------------------------
        lin_x = 0.0
        ang_z = 0.0
        state = STATE_GO
        mode = "person chase"

        # ------------------------------------------------
        # 5. 통합 제어 로직
        # ------------------------------------------------
        if is_close and aligned:
            # 가까움 + 정렬됨 → 완전 정지
            lin_x = 0.0
            ang_z = 0.0
            state = STATE_HOLD
            mode = "stop"

        elif is_close and not aligned:
            # 가까운데 정렬 안 됨 → 회전만
            lin_x = 0.0
            ang_z = clamp(-self.kp_angular * nx, -self.max_ang_vel, self.max_ang_vel)
            mode = "align only"

        else:
            # 아직 멀다 → 접근 + 회전
            lin_x = self.kp_linear * err_area
            lin_x = clamp(lin_x, -self.max_lin_vel, self.max_lin_vel)

            ang_z = clamp(-self.kp_angular * nx, -self.max_ang_vel, self.max_ang_vel)

            mode = f"person chase, area:{area:.3f}"

        # ------------------------------------------------
        # 6. 명령 퍼블리시 (한 번만)
        # ------------------------------------------------
        self._publish_cmd(lin_x, ang_z)
        self._publish_state(state)
        self._publish_mode(mode)

    def _search_mode(self):
        """
        사람(person)을 잃어버렸을 때: 좌/우로 회전하며 찾는 모드
        """
        # 이전 사람이 화면 어느 쪽에 있었는지에 따라 회전 방향 결정
        if self.prev_person_center is not None:
            px, _ = self.prev_person_center
            if px >= self.img_cx:
                # 화면 오른쪽에 있었으면 오른쪽으로 회전 (시계방향)
                ang_z = -abs(self.search_ang_vel)
            else:
                # 왼쪽에 있었으면 왼쪽으로 회전
                ang_z = abs(self.search_ang_vel)
        else:
            # 정보 없으면 한쪽 방향 고정
            ang_z = abs(self.search_ang_vel)

        self._publish_cmd(0.0, ang_z)

        if self.prev_person_center is not None:
            px, _ = self.prev_person_center
            if px >= self.img_cx:
                self._publish_state(STATE_SEARCH_LEFT)
            else:
                self._publish_state(STATE_SEARCH_RIGHT)
            self._publish_mode("finding person")
        else:
            self._publish_state(STATE_SEARCH_LEFT)
            self._publish_mode("finding person")

        self.is_aligned = False


if __name__ == "__main__":
    PersonTrack()
    rospy.spin()
