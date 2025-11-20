#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from yolo11_detect_pkg.msg import Yolo
from sim2real_msg.msg import ControlState
from math import radians

'''
지금 학습 보다는 정지후 공을 찾고
일정시간 움직인 후 다시 공을 찾고
이동중에는 공을 찾지 않게 해봅시다.
'''

# 단순 idea
# 정지한 상태에서 공을 찾기 => 한 바퀴 돌면서 공이 특정 x 범위에 들어오는지 (들어오면 stop)
# 멈추면 일정 시간동안 직진


# rknn으로 공 nms 결과 중 가로축 중앙값 전달 (어짜피 영상을 공 감지 이외의 다른 객체들도 사용하므로 따로 만들어서 정보 전달)
# 가로축 중앙값이 영상의 중앙에서 얼마나 떨어져있냐에 따라 angular.z 값 조종 (rad단위 유의)

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        self.cmd_pub = rospy.Publisher("/cmd_vel_test", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/yolo_state", Int8, queue_size=10)
        self.cmd_vel_msg = Twist()

        self.joy_pub=rospy.Publisher("/joy_msg", ControlState, queue_size=1)
        self.joy_msg=ControlState()

        rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)

        self.img_size = (640, 480)
        self.prev_ball = [0, 0]

        # 상태: 0=직진(RUSH), 1=왼쪽 탐색, 2=오른쪽 탐색, 3=정렬
        self.state = 1

        self.rush_ticks = rospy.get_param("~rush_ticks", 20)  # 기준값 (0.1s 단위)
        self._rush_left = 0  # 실제 카운터

        self.search_ticks=rospy.get_param("~search_ticks", 40)
        self._search_left=0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

    def tracking(self, event):
        if self.state == 0:  # 직진
            if self._rush_left > 0:
                self.cmd_vel_msg.linear.x = 0.5
                self.cmd_vel_msg.angular.z = 0.0
                self._rush_left -= 1
            else:
                # RUSH 끝 → 정지 후 최근 측면으로 탐색
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                if self.prev_ball[0] < self.img_size[0] // 2:
                    self.state = 1
                else:
                    self.state = 2

        elif self.state == 1:  # 왼쪽 탐색
            if self._search_left>0:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 1.0
                self._search_left-=1

        elif self.state == 2:  # 오른쪽 탐색
            if self._search_left>0:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = -1.0
                self._search_left-=1

        elif self.state == 3:  # 정렬 중(회전만)
            self.cmd_vel_msg.linear.x = 0.0
            # angular.z는 calc_direction에서 갱신됨
        
        elif self.state==4:
            self.cmd_vel_msg.linear.x=0.0
            self.cmd_vel_msg.angular.z=0.0

        self.cmd_pub.publish(self.cmd_vel_msg)
        self.state_pub.publish(Int8(self.state))

    def calc_direction(self, xc, yc):
        img_center_x = self.img_size[0] // 2
        diff_horizon = img_center_x - xc  # +면 공이 왼쪽

        if abs(diff_horizon) >= 100:
            # NOTE: 픽셀→라디안은 정확한 변환이 아니지만 기존 스케일 유지
            self.cmd_vel_msg.angular.z = 0.1 * radians(diff_horizon)
            self.state = 3
        else:
            # 정렬 완료 → 일정 시간 직진 시작
            self._rush_left = self.rush_ticks
            self.state = 0

    def yolo_cb(self, msg: Yolo):
        # 이동(직진) 중에는 YOLO 무시
        if self.state == 0:
            return

        found = False
        if msg.count:
            for det in msg.detections:
                if det.label == "Ball":
                    self.calc_direction(det.x_center, det.y_center)
                    self.prev_ball = [det.x_center, det.y_center]
                    found = True
                    break

        if not found:
            self._search_left = self.search_ticks
            # 공이 안 보이면 마지막 위치 기준으로 탐색 방향
            if self.prev_ball[0] < self.img_size[0] // 2:
                self.state = 1
            else:
                self.state = 2

if __name__ == "__main__":
    BallTrack()
    rospy.spin()

