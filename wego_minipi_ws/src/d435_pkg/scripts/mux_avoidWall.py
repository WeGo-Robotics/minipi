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
        rospy.init_node("avoid_wall")
        self.bridge = CvBridge()

        self.cam_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cam_info_cb)
        self.K = None
        self.img_size = None
        self.depth_scale = 0.001

        # 기존 threshold
        # self.OBSTACLE_DISTANCE_M = 0.35

        # -------------------------
        #  히스테리시스 도입
        # -------------------------
        self.OBSTACLE_NEAR = 0.35  # 감지 시작
        self.OBSTACLE_FAR = 0.40  # 감지 종료
        self.obstacle_active = False  # 현재 장애물 상태
        # -------------------------

        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb)
        rospy.loginfo("AvoidWall node started. Waiting for CameraInfo.")

        self.pi_mode = rospy.Publisher("/pi_mode", String, queue_size=10)
        self.cmd_vel_wall = rospy.Publisher("/cmd_vel/wall", Twist, queue_size=10)
        self.cmd_vel_msg = Twist()

    def cam_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape((3, 3))
            self.img_size = [msg.width, msg.height]
            rospy.loginfo(f"CameraInfo received: {self.img_size[0]} * {self.img_size[1]}")
            self.cam_info.unregister()

    def depth_cb(self, msg: Image):
        if self.K is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e:
            rospy.logerr(f"Depth conversion failed: {e}")
            return

        valid_depths = depth_image[depth_image > 0]
        if valid_depths.size == 0:
            rospy.logwarn_throttle(1.0, "No valid depth data")
            return

        H, W = depth_image.shape
        center_y = int(H * 0.4)
        center_line = depth_image[center_y, :]
        valid_depth = center_line[center_line > 0]

        min_depth_m = np.min(valid_depth) * self.depth_scale
        print(f"[Depth] min_depth: {min_depth_m:.3f} m")

        # ---------------------------------------------------
        #  히스테리시스 기반 장애물 ON/OFF 판정
        # ---------------------------------------------------

        # 1) 장애물 새로 감지되는 순간
        if min_depth_m < self.OBSTACLE_NEAR:
            if not self.obstacle_active:
                self.obstacle_active = True
                rospy.loginfo("[STATE] Obstacle ACTIVE (entered)")
            self.avoidance(center_line, W)
            return

        # 2) 장애물 감지 해제되는 순간
        elif min_depth_m > self.OBSTACLE_FAR:
            if self.obstacle_active:
                self.obstacle_active = False
                rospy.loginfo("[STATE] Obstacle CLEARED (exited)")
                self.clear_obstacle()
            return

        # 3) 히스테리시스 구간(0.35~0.40)은 상태 유지
        else:
            if self.obstacle_active:
                self.avoidance(center_line, W)
            return

    def clear_obstacle(self):
        """장애물 해제 시 동작"""
        self.pi_mode.publish("clear")
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_wall.publish(self.cmd_vel_msg)
        rospy.loginfo("[INFO] Obstacle cleared, robot can go forward again.")

    def avoidance(self, center_line: np.ndarray, W: int):
        rospy.loginfo("Obstacle detected! Calculating avoidance maneuver.")

        is_close_obstacle = center_line * self.depth_scale < self.OBSTACLE_NEAR
        obstacle_indices = np.where(is_close_obstacle)[0]

        is_clear = ~is_close_obstacle

        current_start = 0
        max_clear_size = 50
        best_clear_center_idx = W // 2

        for i in range(W + 1):
            if i == W or not is_clear[i]:
                current_end = i
                clear_size = current_end - current_start

                if clear_size > max_clear_size:
                    max_clear_size = clear_size
                    best_clear_center_idx = current_start + clear_size // 2

                current_start = i + 1

        # --------------------
        # 회피 동작 (고정 회전)
        # --------------------
        center_of_image = W // 2

        self.pi_mode.publish("avoid wall")

        # 울타리 기준으로 단순 왼쪽 회전
        angular_z = 0.3

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = angular_z

        rospy.loginfo(f"--> Closest px: {obstacle_indices[0]}-{obstacle_indices[-1]}")
        rospy.loginfo(f"--> Turning {angular_z:.2f} rad/s")

        self.cmd_vel_wall.publish(self.cmd_vel_msg)


if __name__ == "__main__":
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    try:
        AvoidWall()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
