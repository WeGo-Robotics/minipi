#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
# ==============================================================================
# 중요: '/YoloInfo' 토픽의 메시지 타입이 'Yolo'가 맞는지 확인하세요.
# 만약 다르다면, 실제 메시지 타입으로 수정해야 합니다. (예: from yolo_pkg.msg import YoloResult)
from yolo11_detect_pkg.msg import Yolo  # 사용자의 메시지 타입
# ==============================================================================
from sensor_msgs.msg import Image       # 이미지 토픽을 구독하기 위해 추가
from math import pi
class AlignToGoalController:
    def __init__(self):
        """
        클래스 생성자: 노드, 퍼블리셔, 서브스크라이버 및 변수 초기화
        """
        rospy.init_node('align_to_goal_controller', anonymous=True)
        # === 파라미터 및 상태 변수 초기화 ===
        self.image_width = 0
        self.cam_center = 0
        self.ball_center_x = None
        self.goal_center_x = None
        # --- [추가됨] 제어 파라미터 ---
        self.Kp_z = 0  # 각속도(angular.z)를 위한 P 게인 (자동 계산)
        self.Kp_y = 0  # y축 선속도(linear.y)를 위한 P 게인 (자동 계산)
        self.MAX_LINEAR_Y = 1.0  # 로봇의 최대 좌우 이동 속도 (m/s), 사용자가 튜닝할 핵심 파라미터
        self.DEAD_ZONE_PIXELS = 5  # 이 픽셀 오차 범위 내에서는 움직이지 않음 (모터 떨림 방지)
        # ---------------------------
        # === ROS Publisher & Subscriber 설정 ===
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/YoloInfo', Yolo, self.yolo_callback)
        self.cam_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.cam_callback, tcp_nodelay=True)
        self.cmd_msg = Twist()
        self.rate = rospy.Rate(10)
        rospy.loginfo("AlignToGoalController node has been initialized.")
    def cam_callback(self, msg: Image):
        """
        카메라 토픽을 1회 구독하여 제어에 필요한 초기값을 설정합니다.
        """
        if self.image_width == 0:
            self.image_width = msg.width
            self.image_height = msg.height
            self.cam_center = self.image_width // 2
            rospy.loginfo("Image size received: %d x %d", self.image_width, self.image_height)
            # [수정됨] 각속도(Kp)와 선속도(Kp_y) P제어 게인을 물리적 의미를 담아 자동 계산
            if self.image_width > 0:
                self.Kp_z = pi / self.image_width  # 각속도 Kp
            if self.cam_center > 0:
                self.Kp_y = self.MAX_LINEAR_Y / self.cam_center  # y축 선속도 Kp
                rospy.loginfo(f"Kp for angular.z set to: {self.Kp_z:.4f}")
                rospy.loginfo(f"Kp_y for linear.y set to: {self.Kp_y:.4f}")
            self.cam_sub.unregister()
            rospy.loginfo("Camera subscriber unregistered.")
    def yolo_callback(self, data: Yolo):
        """
        YOLO 토픽을 구독하여 'ball'과 'goal'의 화면상 x좌표를 업데이트합니다.
        """
        current_ball_x = None
        current_goal_x = None
        for detection in data.detections:
            if detection.label == 'ball':
                current_ball_x = detection.x_center
            elif detection.label == 'red_goal':
                current_goal_x = detection.x_center
        self.ball_center_x = current_ball_x
        self.goal_center_x = current_goal_x
    def move_robot(self):
        """
        탐지된 객체 상태에 따라 로봇의 선속도/각속도를 계산하고 발행하는 메인 루프.
        """
        rospy.loginfo("Waiting for image size from camera topic...")
        while self.image_width == 0 and not rospy.is_shutdown():
            self.rate.sleep()
        rospy.loginfo("Controller started. Robot will now move based on detections.")
        while not rospy.is_shutdown():
            linear_x = 0.0
            linear_y = 0.0
            angular_z = 0.0
            # 상태 1: 'ball'과 'goal'이 모두 보일 때
            if self.ball_center_x is not None and self.goal_center_x is not None:
                dist_error = self.ball_center_x - self.goal_center_x
                angle_error = self.cam_center - self.ball_center_x
                # --- [수정됨] linear_y 계산 로직 ---
                # 1. 데드존 처리: 오차가 매우 작으면 움직이지 않음
                if abs(dist_error) < self.DEAD_ZONE_PIXELS:
                    linear_y = 0.0
                else:
                    # 2. P 제어 적용: 계산된 Kp_y를 사용해 속도 계산
                    linear_y = -self.Kp_y * dist_error
                    angular_z = -self.Kp_z * angle_error
                    # 3. 속도 제한 (Clipping): 계산된 속도가 최대 속도를 넘지 않도록 함
                    linear_y = max(-self.MAX_LINEAR_Y, min(linear_y, self.MAX_LINEAR_Y))
                # ------------------------------------
                linear_x = 0.0
                angular_z = -self.Kp_z * angle_error * 0.1
                rospy.loginfo(f"State 1: Aligning. dist_error: {dist_error}, linear_y: {linear_y:.3f}")
            # 상태 2: 'ball'만 보일 때
            elif self.ball_center_x is not None:
                angle_error = self.cam_center - self.ball_center_x
                angular_z = -self.Kp_z * angle_error
                linear_x = 0.5
                linear_y = 0.0
                rospy.loginfo(f"State 2: Centering on ball. angle_error: {angle_error}, angular_z: {angular_z:.3f}")
            # 상태 3: 'ball'이 보이지 않을 때
            else:
                linear_x = -0.2
                linear_y = 0.0
                angular_z = 0.2
                rospy.loginfo("State 3: Searching for ball...")
            self.cmd_msg.linear.x = linear_x
            self.cmd_msg.linear.y = linear_y
            self.cmd_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(self.cmd_msg)
            self.rate.sleep()
if __name__ == '__main__':
    try:
        controller = AlignToGoalController()
        controller.move_robot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        pass