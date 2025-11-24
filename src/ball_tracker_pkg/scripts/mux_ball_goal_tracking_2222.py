#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
# ==============================================================================
# 메시지 타입 확인: 'Yolo' 메시지 타입을 사용합니다.
from yolo11_detect_pkg.msg import Yolo  # 사용자의 메시지 타입
# ==============================================================================
from sensor_msgs.msg import Image
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
        
        # --- [제어 및 튜닝 파라미터] ---
        self.Kp_z = 0  # 각속도(angular.z)를 위한 P 게인 (자동 계산)
        self.Kp_y = 0  # y축 선속도(linear.y)를 위한 P 게인 (자동 계산)
        self.MAX_LINEAR_Y = 1.0          # 로봇의 최대 좌우 이동 속도 (m/s)
        self.DEAD_ZONE_PIXELS = 5        # 이 픽셀 오차 범위 내에서는 움직이지 않음
        
        # --- 🎯 [정렬 및 슛팅 관련 파라미터] ---
        self.ALIGN_TOLERANCE_PIXELS = 10 # 정렬 완료로 판단할 ball과 goal 사이의 최대 픽셀 오차
        self.ALIGN_CENTER_TOLERANCE_PIXELS = 10 # ball의 화면 중앙 오차 허용 범위
        self.SHOOT_LINEAR_X = 1.0        # 골을 넣을 때의 전진 속도 (m/s)
        self.is_aligned = False          # 정렬 완료 플래그 (새로 추가)
        # ------------------------------------

        # === ROS Publisher & Subscriber 설정 ===
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/YoloInfo', Yolo, self.yolo_callback)
        # 카메라 정보를 한 번만 구독하여 초기값을 설정합니다.
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
            
            # 각속도(Kp)와 선속도(Kp_y) P제어 게인을 물리적 의미를 담아 자동 계산
            if self.image_width > 0 and self.cam_center > 0:
                self.Kp_z = pi / self.image_width  # 각속도 Kp
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
            elif detection.label == 'goal':
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
            
            # ⚽ 상태 4: 정렬 완료 후 직진/슛팅 (새로운 상태)
            if self.is_aligned:
                linear_x = self.SHOOT_LINEAR_X
                angular_z = 0.0
                linear_y = 0.0
                rospy.loginfo(f"State 4: Aligned! SHOOTING with linear_x: {linear_x:.3f}")
                
            # 🥅 상태 1: 'ball'과 'goal'이 모두 보일 때 (정렬 시도)
            elif self.ball_center_x is not None and self.goal_center_x is not None:
                dist_error = self.ball_center_x - self.goal_center_x   # ball과 goal 간의 오차
                angle_error = self.cam_center - self.ball_center_x     # ball과 화면 중앙 간의 오차
                
                # --- 🎯 정렬 완료 조건 체크 ---
                # 1. ball과 goal의 정렬 오차가 허용 범위 이내이고
                # 2. ball이 화면 중앙으로부터의 오차가 허용 범위 이내일 때
                if abs(dist_error) < self.ALIGN_TOLERANCE_PIXELS and \
                   abs(angle_error) < self.ALIGN_CENTER_TOLERANCE_PIXELS:
                    
                    self.is_aligned = True # 정렬 완료 플래그 설정
                    # 이번 루프에서는 직진 명령을 주고 다음 루프에서 State 4로 전환됩니다.
                    linear_x = self.SHOOT_LINEAR_X
                    rospy.loginfo("State 1 -> Aligned! Transition to SHOOTING.")
                
                # --- 정렬 중 ---
                else:
                    self.is_aligned = False
                    
                    # linear_y (좌우 이동) 제어
                    if abs(dist_error) > self.DEAD_ZONE_PIXELS:
                        linear_y = -self.Kp_y * dist_error
                        linear_y = max(-self.MAX_LINEAR_Y, min(linear_y, self.MAX_LINEAR_Y))
                    else:
                        linear_y = 0.0
                    
                    # angular_z (회전) 제어 (볼을 중앙에 맞추려 함)
                    angular_z = -self.Kp_z * angle_error * 0.1
                    
                    linear_x = 0.0 # 정렬 중에는 전진하지 않음
                    rospy.loginfo(f"State 1: Aligning. dist_error: {dist_error:.1f}, linear_y: {linear_y:.3f}")

            # 🏃 상태 2: 'ball'만 보일 때 (추적/중앙 맞추기)
            elif self.ball_center_x is not None:
                self.is_aligned = False
                angle_error = self.cam_center - self.ball_center_x
                angular_z = -self.Kp_z * angle_error
                linear_x = 0.3
                linear_y = 0.0
                rospy.loginfo(f"State 2: Centering on ball. angle_error: {angle_error}, angular_z: {angular_z:.3f}")
                
            # 🔍 상태 3: 'ball'이 보이지 않을 때 (탐색)
            else:
                self.is_aligned = False
                linear_x = -0.2
                linear_y = 0.0
                angular_z = 0.2
                rospy.loginfo("State 3: Searching for ball...")
                
            # --- 명령 발행 ---
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