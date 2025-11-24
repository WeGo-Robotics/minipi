#!/usr/bin/env python3
import rospy
import numpy as np
import os
import cv2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AvoidWall:
    def __init__(self):
        rospy.init_node('avoid_wall')
        self.bridge = CvBridge()

        self.cam_info=rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cam_info_cb)
        self.K = None
        self.img_size=None
        self.depth_scale = 0.001
        self.OBSTACLE_DISTANCE_M=0.35
        
        self.depth_sub=rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb)
        rospy.loginfo("AvoidWall node started. Waiting for CameraInfo.")

        self.pi_mode=rospy.Publisher("/pi_mode", String, queue_size=10)
        self.cmd_vel_wall=rospy.Publisher("/cmd_vel/wall", Twist, queue_size=10)
        self.cmd_vel_msg=Twist()

    def cam_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape((3, 3))
            self.img_size=[msg.width, msg.height]
            rospy.loginfo(f"CameraInfo received: {self.img_size[0]} * {self.img_size[1]}")
            self.cam_info.unregister() # 정보는 한 번만 필요
    
    def depth_cb(self, msg: Image):
        if self.K is None:
            return
            
        try:
            # 1. CvBridge를 사용하여 16비트 깊이 이미지(mm)로 변환
            # RealSense 깊이 인코딩은 16UC1로 명시
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # 2. 유효한 깊이 값 추출 (0보다 큰 값)
            valid_depths = depth_image[depth_image > 0]
            
            if valid_depths.size == 0:
                rospy.logwarn_throttle(1.0, "No valid depth data (>0) received.")
                return

            # --- 핵심 로직: 정면 벽과의 거리 계산 ---
            
            # 이미지 중앙을 포함하는 영역 (예: 너비 1/3, 높이 중앙 1/5)
            H, W = depth_image.shape
            
            # 중앙 영역 정의: 중앙 1/3 너비, 중앙 높이 (480//2)
            # 실제로 현장에서 울타리가 어느정도 높이일 지 알 수 없음, 유동적으로 조정
            center_y = int(H*0.4) # y 값을 고정

            # LiDAR 처럼 640개의 값을 쓸 지, 중앙 값을 쓸 지...
            center_x_start = W * 2// 5
            center_x_end = W * 3 // 5
            
            # 중앙 가로줄 추출
            # center_line = depth_image[center_y, center_x_start:center_x_end]
            center_line = depth_image[center_y, :]
            
            # 유효한 중앙 깊이 값
            valid_depth = center_line[center_line > 0]
            
            # 영역 중 최소 거리 찾기
            # if valid_center_depths.size > 0:
            #     # 중앙 영역의 최소 거리 (가장 가까운 벽)
            #     min_depth_mm = np.min(valid_center_depths)
            #     min_depth_m = min_depth_mm * self.depth_scale
                
            #     print(f"정면 중앙 최소 거리: {min_depth_m:.3f} m")

            #     # 480//2 높이에서 거리 d 미만이면 다른 곳으로 회피
            #     if min_depth_m < 0.2: # 예: 0.5m 미만이면 회피
            #         self.avoidance() 

            # 영역 중 이동 공간 넓은 곳 찾기
            min_depth_m = np.min(valid_depth) * self.depth_scale
            # print(f"center_y에서 depth 값들: {center_line}")
            print(f"정면 중앙 최소 거리: {min_depth_m:.3f} m")

            # --- 2. 장애물 판단 및 회피 로직 ---
            if min_depth_m < self.OBSTACLE_DISTANCE_M:
                # 장애물 감지!
                self.avoidance(center_line, W)
            else:
                # 장애물이 충분히 멀리 있음 -> 전진
                # self.cmd_vel_msg.linear.x = 0.2
                # self.cmd_vel_msg.angular.z = 0.0
                # self.cmd_vel_wall.publish(self.cmd_vel_msg)
                pass

            # else:
            #     rospy.logwarn_throttle(1.0, "Central region has no valid depth data.")

        except Exception as e:
            rospy.logerr(f"Depth data processing failed: {e}")
            
    def avoidance(self, center_line: np.ndarray, W: int):
        rospy.loginfo("Obstacle detected! Calculating refined avoidance maneuver.")

        # 1. threshold 미만인 픽셀만 '장애물'로 간주 (depth == 0, 즉 측정 실패/무한대는 무시)
        # d435가 .3m~3m 까지 지원하므로 0이면 아주 가깝거나 아주 먼 것
        is_close_obstacle = (center_line * self.depth_scale < self.OBSTACLE_DISTANCE_M)
        
        # 2. 장애물 픽셀 인덱스 추출
        obstacle_indices = np.where(is_close_obstacle)[0] # 장애물이라고 판단한 픽셀들 나옴 (depth는 width 848)
        # print(f"is_close_obstacle: {np.where(is_close_obstacle)[0]}")
        
        # if obstacle_indices.size == 0:
        #     rospy.logwarn_throttle(1.0, "Close obstacle vanished. Resuming forward movement.")
        #     self.cmd_vel_msg.linear.x = 0.2
        #     self.cmd_vel_msg.angular.z = 0.0
        #     self.cmd_vel_wall.publish(self.cmd_vel_msg)
        #     return
            
        # 3. 가장 넓은 공간 찾기
        
        # 장애물이 아닌 (depth > 0.2m) 픽셀을 True로 설정
        is_clear = ~is_close_obstacle
        
        current_start = 0
        max_clear_size = 50 # 임계값 설정
        best_clear_center_idx = W // 2 # 기본값은 중앙

        # 가장 긴 'clear' 구간(True) 찾기
        for i in range(W + 1): # W까지 검사하여 마지막 구간 처리
            # 이미지 끝에 도달했거나, clear 구간이 종료되면 (i-1이 장애물)
            if i == W or not is_clear[i]:
                current_end = i
                clear_size = current_end - current_start
                
                if clear_size > max_clear_size:
                    max_clear_size = clear_size
                    # 가장 넓은 빈 공간의 중앙 인덱스 계산
                    best_clear_center_idx = current_start + clear_size // 2
                
                # 다음 탐색 시작점을 현재 픽셀(장애물) 다음 픽셀로 설정
                current_start = i + 1
        
        # 4. 회전 방향 및 속도 설정 (이전과 동일)
        turn_angle_min=0.4 # 최소 회전 속도
        turn_angle_max = 0.8 # 최대 회전 속도 (Rad/s)
        
        center_of_image = W // 2
        if center_of_image != 0: 
            self.pi_mode.publish("avoid wall")
            angular_z=0.3

        # pixel_diff = best_clear_center_idx - center_of_image
        
        # # 회전 속도 정규화: (픽셀 차이 / 이미지 반폭) * 최대 속도
        # # 굳이 회전 속도 정규화를 할 필요는 없어보임
        # # (center_of_image가 0이 아닐 경우)
        # if center_of_image != 0:
        #     # 1. 비례값 계산 (부호 반전: 오른쪽 빈 공간(양수) -> 오른쪽 회전(음수))
        #     raw_angular_z = turn_angle_max * (-pixel_diff / center_of_image)
            
        #     if abs(raw_angular_z) < turn_angle_min:
        #         angular_z = turn_angle_min * np.sign(raw_angular_z)
        #     elif abs(raw_angular_z) > turn_angle_max:
        #         angular_z = turn_angme_max * np.sign(raw_angular_z)
        #     else:
        #         # 최소/최대 범위 안에 있으면 그대로 사용
        #         angular_z = raw_angular_z
            
        #     #  각속도 고정하는 것이 나음. (로보월드 환경에서는 방향도 나눌 필요 없어보임)
        #     if pixel_diff < 0:
        #         angular_z = 0.5
        #     else:
        #         angular_z = -0.5


        # else: # 이미지 너비가 0인 경우는 없겠지만 안전을 위해
        #      angular_z = turn_angle_max 
        
        # 선형 속도는 0으로 설정하여 회피에 집중
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = angular_z

        rospy.loginfo(f"--> Closest Pxl: {obstacle_indices[0]}-{obstacle_indices[-1]}, Clear Pxl: {best_clear_center_idx} (Size: {max_clear_size})")
        rospy.loginfo(f"--> Turning towards {angular_z:.1f} rad/s")

        self.cmd_vel_wall.publish(self.cmd_vel_msg)

if __name__=='__main__':
    # GUI 충돌 방지 환경 변수 설정
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    try:
        AvoidWall()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass