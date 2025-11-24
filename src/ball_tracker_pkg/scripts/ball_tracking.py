#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from math import radians
from sim2real_msg.msg import usr2ctrl

class BallTrack:
    def __init__(self):
        rospy.init_node("minipi_ball_tracking")

        self.ctrl_pub=rospy.Publisher("/usr2ctrl_data", usr2ctrl, queue_size=10)
        self.ctrl_msg=usr2ctrl()
        self.ctrl_msg.robot_status = 4

        self.yolo_sub = rospy.Subscriber("/YoloInfo", Yolo, self.yolo_cb)
        self.img_size = (640, 480)  # (W, H)
        self.prev_ball = None       # [x_center, y_center]
        self.frame_cnt = 0
        self.state = 1              # 0: 추종, 1: 왼쪽 탐색, 2: 오른쪽 탐색

        # '정면 근접 실종' 처리용: 정면 판정 여유(px)와 블라인드 전진 주기 수
        self.center_margin = rospy.get_param("~center_margin_px", 40)
        self.blind_forward_ticks = rospy.get_param("~blind_forward_ticks", 5)
        self._blind_cnt = 0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

    def tracking(self, event):
        if self.state == 0:
            # 추종 중: yolo_cb / calc_direction 에서 cmd_vel_msg 갱신됨
            pass
        elif self.state == 1:
            self.ctrl_msg.twist_data.linear.x  = 0.0
            self.ctrl_msg.twist_data.angular.z = 1.0
        elif self.state == 2:
            self.ctrl_msg.twist_data.linear.x  = 0.0
            self.ctrl_msg.twist_data.angular.z = -1.0
        self.ctrl_pub.publish(self.ctrl_msg)

    def calc_direction(self, xc, yc):
        img_center_x = self.img_size[0] // 2
        self.ctrl_msg.twist_data.linear.x  = 0.3
        # 요청 로직 유지: 픽셀을 'deg'처럼 간주해 라디안 스케일로 축소(대략적인 스케일링)
        self.ctrl_msg.twist_data.angular.z = 0.1 * radians(img_center_x - xc)

    def yolo_cb(self, msg: Yolo):
        count = msg.count
        saw_ball = False

        if count:
            for dtc_info in msg.detections:
                if dtc_info.label == "Ball":
                    self.calc_direction(dtc_info.x_center, dtc_info.y_center)
                    self.prev_ball = [dtc_info.x_center, dtc_info.y_center]
                    self.state = 0
                    saw_ball = True
                    break  # 첫 번째 Ball만 사용

        # 프레임 카운트 처리: 공을 '봤을 때만' 리셋
        if saw_ball:
            self.frame_cnt = 0
            self._blind_cnt = 0
        else:
            self.frame_cnt += 1

            # ----- 공이 발 앞에 있어 카메라에 안 잡힐 때(정면 근처에서 실종) 처리 -----
            if self.frame_cnt > 10:
                if self.prev_ball is not None:
                    cx = self.img_size[0] // 2
                    # 마지막 x가 중심 근처면 잠깐 '블라인드 전진'
                    if abs(self.prev_ball[0] - cx) <= self.center_margin and self._blind_cnt < self.blind_forward_ticks:
                        self.ctrl_msg.twist_data.linear.x  = 0.1
                        self.ctrl_msg.twist_data.angular.z = 0.0
                        self.state = 0  # 계속 직진 명령을 퍼블리시하게 함
                        self._blind_cnt += 1
                    else:
                        # 좌/우로 사라진 경우 탐색 회전
                        if self.prev_ball[0] < cx:
                            self.state = 1
                        else:
                            self.state = 2
                else:
                    # 히스토리 자체가 없으면 기본 왼쪽 탐색
                    self.state = 1

if __name__ == "__main__":
    BallTrack()
    rospy.spin()
