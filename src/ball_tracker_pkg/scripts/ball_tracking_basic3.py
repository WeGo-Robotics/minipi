#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from yolo11_detect_pkg.msg import Yolo
from sim2real_msg.msg import ControlState
import numpy as np

# 상태: 0=GO(이동 3초), 4=HOLD(정지 대기)
GO, HOLD = 0, 4

# 공 크기 기준으로 거리 가늠 => 거리에 비례해서 이동하도록
# 로봇 step에 맞춰서 정지할 수 있도록

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        # Pubs/Subs
        self.cmd_pub   = rospy.Publisher("/cmd_vel_test", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state",   Int8,  queue_size=10)
        self.joy_pub   = rospy.Publisher("/joy_msg", ControlState, queue_size=10)

        rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)

        # 메시지 버퍼
        self.cmd = Twist()
        self.joy = ControlState()

        # 이미지 크기(센터 계산용)
        self.img_w, self.img_h = 640, 480

        # 파라미터
        self.rush_ticks    = rospy.get_param("~rush_ticks", 30)       # 0.1s 단위 → 30 = 3초
        self.forward_speed = rospy.get_param("~forward_speed", 1.0)   # GO 동안 linear.x (기본값)
        self.angular_gain  = rospy.get_param("~angular_gain", 0.02)  # x오차(픽셀) → rad/s 변환 게인

        # 내부 상태
        self.state      = HOLD
        self._go_left   = 0            # GO 남은 틱
        self._walk_flag = False        # Joy 토글 추적: False=정지, True=보행

        # 주기 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)

    def loop(self, _evt):
        if self.state == GO:
            if self._go_left > 0:
                self._go_left -= 1
                # self.cmd 는 GO 진입 시 고정값으로 셋됨(아래 yolo_cb)
                # 여기선 카운트 다운만
            else:
                # GO 종료 → 완전 정지(HOLD)
                self._enter_hold()

        elif self.state == HOLD:
            # 완전 정지로 유지
            self.cmd.linear.x  = 0.0
            self.cmd.angular.z = 0.0

        # Joy 토글: HOLD↔GO 전이에만 1 펄스
        desired_walk = (self.state == GO)
        if desired_walk != self._walk_flag:
            self.joy.running_standby_switch = 1.0
            self._walk_flag = desired_walk
        else:
            self.joy.running_standby_switch = 0.0

        # Publish
        self.cmd_pub.publish(self.cmd)
        self.state_pub.publish(Int8(self.state))
        self.joy_pub.publish(self.joy)

    def _enter_hold(self):
        self.state = HOLD
        self.cmd.linear.x  = 0.0
        self.cmd.angular.z = 0.0
        self._go_left = 0

    def _enter_go_with(self, v, w):
        self.state = GO
        self._go_left = self.rush_ticks
        # GO 동안에는 고정된 v/ω를 유지(3초)
        self.cmd.linear.x  = float(v)
        self.cmd.angular.z = float(w)

    def yolo_cb(self, msg: Yolo):
        """ HOLD일 때만 사용. 공이 보이면 그 순간의 bbox로 v/ω 계산해서 3초 GO 진입 """
        if self.state != HOLD:
            return  # GO 중에는 YOLO 무시

        # 공 탐색
        if not msg.count:
            return

        det_ball = None
        for det in msg.detections:
            if (det.label or "").lower() == "ball":
                det_ball = det
                break
        if det_ball is None:
            return

        # v/ω 계산 (그 순간 값으로 고정)
        # v: 고정 속도(파라미터), w: 화면 중심 대비 x오차 비례
        v_max = 1.0
        v_min = rospy.get_param("~v_min", 0.05)

        ball_size = (det_ball.w * det_ball.h) / float(self.img_w * self.img_h)
        if ball_size>=10000:
            # ball_size가 10000이상이면 무시하도록 (검증 용도)
            pass

        # bbox의 중심 y좌표도 사용하여 속도 조절
        # 결론: bbox 크기: 공 검증, bbox 중심 x좌표: angular velocity, bbox 중심 y좌표: linear velocity
        
        # det_ball.y_center
        # self.img_h-margin 에서 margin을 지정하면 공이 가까워질 때를 알 수 있을 것
        
        
        v = v_min + (v_max - v_min)
        v = float(np.clip(v, v_min, v_max))

        # w 계산 (정규화 + 포화)
        w_max = rospy.get_param("~w_max", 1.2)
        cx = self.img_w // 2
        e = (cx - det_ball.x_center) / float(self.img_w/2)
        w = float(np.clip(w_max * e, -w_max, w_max))

        print(f"ball_size={ball_size:.4f}, v={v:.3f}, w={w:.3f}")
        self._enter_go_with(v, w)

if __name__ == "__main__":
    BallTrack()
    rospy.spin()
