#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from yolo11_detect_pkg.msg import Yolo
from math import radians

# 상태 상수: 0=GO(RUSH), 1=SEARCH_LEFT, 2=SEARCH_RIGHT, 3=CALIB(ALIGN), 4=HOLD
GO, SLEFT, SRIGHT, CALIB, HOLD = 0, 1, 2, 3, 4

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        self.cmd_pub   = rospy.Publisher("/cmd_vel_test", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state",   Int8,  queue_size=10)
        self.cmd_vel_msg = Twist()

        rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)

        self.img_size  = (640, 480)
        self.prev_ball = [0, 0]

        # 파라미터
        self.rush_ticks    = rospy.get_param("~rush_ticks",    20)  # 0.1s 단위
        self.search_ticks  = rospy.get_param("~search_ticks",  40)  # 0.1s 단위 (4초)
        self.forward_speed = rospy.get_param("~forward_speed", 0.5)

        # 내부 카운터
        self._rush_left   = 0
        self._search_left = 0

        # 초기 상태: HOLD
        self.state = HOLD

        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

    def _start_one_search(self):
        # 마지막 공 위치 기준으로 1회 회전 탐색 시작
        if self.prev_ball[0] < self.img_size[0] // 2:
            self.state = SLEFT
        else:
            self.state = SRIGHT
        self._search_left = self.search_ticks

    def tracking(self, event):
        if self.state == GO:
            if self._rush_left > 0:
                self.cmd_vel_msg.linear.x  = self.forward_speed
                self.cmd_vel_msg.angular.z = 0.0
                self._rush_left -= 1
            else:
                # 직진 종료 → HOLD로 정지
                self._enter_hold()

        elif self.state == SLEFT:
            if self._search_left > 0:
                self.cmd_vel_msg.linear.x  = 0.0
                self.cmd_vel_msg.angular.z = 1.0
                self._search_left -= 1
            else:
                # 4초 검색 종료 → HOLD
                self._enter_hold()

        elif self.state == SRIGHT:
            if self._search_left > 0:
                self.cmd_vel_msg.linear.x  = 0.0
                self.cmd_vel_msg.angular.z = -1.0
                self._search_left -= 1
            else:
                # 4초 검색 종료 → HOLD
                self._enter_hold()

        elif self.state == CALIB:
            self.cmd_vel_msg.linear.x  = 0.0
            # angular.z는 calc_direction에서 갱신됨

        elif self.state == HOLD:
            self.cmd_vel_msg.linear.x  = 0.0
            self.cmd_vel_msg.angular.z = 0.0

        self.cmd_pub.publish(self.cmd_vel_msg)
        self.state_pub.publish(Int8(self.state))

    def _enter_hold(self):
        self.state = HOLD
        self.cmd_vel_msg.linear.x  = 0.0
        self.cmd_vel_msg.angular.z = 0.0

    def calc_direction(self, xc, yc):
        img_center_x = self.img_size[0] // 2
        diff_horizon = img_center_x - xc  # +면 공이 왼쪽

        if abs(diff_horizon) >= 100:
            # 간단 스케일(픽셀→라디안 아님)
            self.cmd_vel_msg.angular.z = 0.1 * radians(diff_horizon)
            self.state = CALIB
        else:
            # 정렬 완료 → 일정 시간 직진 시작
            self._rush_left = self.rush_ticks
            self.state = GO

    def yolo_cb(self, msg: Yolo):
        # 직진 중에는 YOLO 무시
        if self.state == GO:
            return

        found = False
        if msg.count:
            for det in msg.detections:
                if (det.label or "").lower() == "ball":
                    self.calc_direction(det.x_center, det.y_center)
                    self.prev_ball = [det.x_center, det.y_center]
                    found = True
                    break

        if not found:
            # HOLD에서는 검출될 때까지 가만히 대기
            if self.state == HOLD:
                # 자동으로 바로 검색 시작하지 않음
                return
            # 검색/정렬 중이면 타이머와 상태에 맡김(검색 타이머 끝나면 HOLD로 전환)

if __name__ == "__main__":
    BallTrack()
    rospy.spin()
