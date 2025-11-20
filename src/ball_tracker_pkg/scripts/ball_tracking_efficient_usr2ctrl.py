#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from yolo11_detect_pkg.msg import Yolo
from sim2real_msg.msg import usr2ctrl
import numpy as np

GO, HOLD = 0, 4

# 1=실행, 0=정지
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0

# pixel to distance 인자 (y기준)
linear = (-0.010895, 6.024430)
rational = (-84.166458, 50689.309424, 25.534673, 1)

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        self.ctrl_pub=rospy.Publisher("/usr2ctrl_data", usr2ctrl, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state",   Int8,  queue_size=10)
        self.control_pub = rospy.Publisher("/yolo_run_control", Int8, queue_size=1)

        rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)

        self.ctrl_msg=usr2ctrl()

        self.img_w, self.img_h = 640, 480

        # 타이밍 파라미터
        self.rush_ticks = rospy.get_param("~rush_ticks", 30)     # 0.1s 단위 → 3초 주행
        self.arm_ticks  = rospy.get_param("~arm_ticks", 10)      # 0.1s 단위 → 1초 대기(검출 후)

        # 선속도 매핑(v)
        self.v_min = rospy.get_param("~v_min", 0.1)
        self.v_max = rospy.get_param("~v_max", 0.8)              # 최대 선속도 0.8
        self.s_ref = rospy.get_param("~s_ref", 0.10)
        self.k_exp = rospy.get_param("~k_exp", 3.0)

        # 각속도 매핑(w)
        self.w_max      = rospy.get_param("~w_max", 1.0)         # 각속도 범위 [-1, 1]
        self.w_deadband = rospy.get_param("~w_deadband", 0.1)   # 정규화 오차 데드밴드
        self.w_gamma    = rospy.get_param("~w_gamma", 5.0)       # 비선형 지수(↑=더 완만)
        self.w_offset = rospy.get_param("~w_offset", 0.35)

        # 내부 상태
        self.state        = HOLD
        self._go_left     = 0
        self._arm_left    = 0           # 검출 후 출발까지 남은 대기 틱
        self._pending_vw  = None        # (v,w) 저장해서 대기 끝에 사용

        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)

    def pixel_to_distance(self, y_pix, method):
        if len(method) == 2:
            return method[0] * y_pix + method[1]
        else:
            return (method[0] * y_pix + method[1]) / (method[2] * y_pix + method[3])


    def loop(self, _):
        # 대기(arming) 처리: HOLD에서만 카운트다운 → 0되면 출발
        if self.state == HOLD:
            if self._arm_left > 0:
                self._arm_left -= 1
                # 완전 정지 유지
                self.ctrl_msg.twist_data.linear.x  = 0.0
                self.ctrl_msg.twist_data.angular.z = 0.0
                # 대기 끝 + 보류된 명령이 있으면 GO 진입
                if self._arm_left == 0 and self._pending_vw is not None:
                    v, w = self._pending_vw
                    self._pending_vw = None
                    self._enter_go_with(v, w)
            else:
                # 대기 아님 → 완전 정지
                self.ctrl_msg.twist_data.linear.x  = 0.0
                self.ctrl_msg.twist_data.angular.z = 0.0

        elif self.state == GO:
            if self._go_left > 0:
                self._go_left -= 1
            else:
                self._enter_hold()

        # self.ctrl_msg.robot_status는 _enter_hold/go_with에서 설정됩니다.

        # GO 상태일 때 (0) -> YOLO 정지 (0)
        # HOLD 상태일 때 (4) -> YOLO 실행 (1)
        yolo_control_value = YOLO_CONTROL_STOP if self.state == GO else YOLO_CONTROL_RUN
        self.control_pub.publish(Int8(yolo_control_value))

        self.ctrl_pub.publish(self.ctrl_msg)
        self.state_pub.publish(Int8(self.state))

    def _enter_hold(self):
        self.state = HOLD
        self.ctrl_msg.twist_data.linear.x  = 0.0
        self.ctrl_msg.twist_data.angular.z = 0.0
        self.ctrl_msg.robot_status = 3
        self._go_left    = 0
        # 출발 대기는 유지/리셋 둘 다 가능하지만, 여기서는 유지하지 않음
        # self._arm_left = 0
        # self._pending_vw = None

    def _enter_go_with(self, v, w):
        self.state = GO
        self._go_left = self.rush_ticks
        self.ctrl_msg.twist_data.linear.x  = float(v)
        self.ctrl_msg.twist_data.angular.z = float(w)
        self.ctrl_msg.robot_status = 4

    def yolo_cb(self, msg: Yolo):
        # GO 중에는 YOLO 무시 (2차 방어 로직: 제어 신호가 실패해도 데이터 처리는 막음)
        if self.state != HOLD:
            return
        if not msg.count:
            return

        # 이미 arming 중이면 다시 무장하지 않음(깜빡임 방지)
        if self._arm_left > 0 or self._pending_vw is not None:
            return

        # 첫 번째 Ball만 사용
        det_ball = next((d for d in msg.detections if (d.label or "").lower() == "ball"), None)
        if det_ball is None:
            return
        
        # pixel to distance 인자 (y기준)
        # 이미 클래스 레벨에서 정의된 linear/rational을 사용해야 하지만, 로컬 변수가 재정의되어 있었으므로 유지
        linear_local = (-0.010895, 6.024430)
        rational_local = (-84.166458, 50689.309424, 25.534673, 1)

        print(f"det_ball.y_center: {det_ball.y_center}")
        distance_m=self.pixel_to_distance(det_ball.y_center, rational_local)
        print(f"distance (meter): {distance_m}")

        # --- v 계산: 거리(m) 만큼 선속도 지정(m/s 단위) ---
        v = min(self.v_max, max(self.v_min, distance_m))

        # --- w 계산 (데드밴드 + 비선형 완만) ---
        cx = self.img_w // 2
        e  = (cx - det_ball.x_center) / float(self.img_w/2)  # -1..1
        if abs(e) <= self.w_deadband:
            e_eff = 0.0
        else:
            e_sign = np.sign(e)
            e_mag  = (abs(e) - self.w_deadband) / (1.0 - self.w_deadband)
            e_eff  = e_sign * (e_mag ** self.w_gamma)
        
        w_raw = self.w_max * e_eff
        w = float(np.clip(w_raw + self.w_offset, -self.w_max, self.w_max))

        # 출발 대기(arming) 시작: 1초 후 저장한 v,w로 출발
        self._pending_vw = (v, w)
        self._arm_left   = self.arm_ticks

if __name__ == "__main__":
    BallTrack()
    rospy.spin()
