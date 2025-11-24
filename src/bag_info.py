#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy
from tf.transformations import euler_from_quaternion

BAG_PATH = "src/run_2025-10-02_0.bag"
IMG_TOPIC = "/usb_cam/image_raw"     # 저장할 때 쓴 토픽명
IMU_TOPIC = "/imu/data"              # 표준화된 IMU 사용
TIME_TOL = 0.02                      # 20ms 이내를 같은 프레임으로 간주

def main():
    bag = rosbag.Bag(BAG_PATH, "r")
    bridge = CvBridge()

    imu_msgs = []
    for topic, msg, t in bag.read_messages(topics=[IMU_TOPIC]):
        imu_msgs.append((t.to_sec(), msg))

    print(f"[INFO] Loaded {len(imu_msgs)} IMU messages")

    for topic, msg, t in bag.read_messages(topics=[IMG_TOPIC]):
        # ROS Image → OpenCV
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_time = t.to_sec()

        # 가까운 IMU 찾기
        closest = min(imu_msgs, key=lambda x: abs(x[0] - img_time))
        dt = abs(closest[0] - img_time)
        imu_msg = closest[1]

        print(f"\n=== Frame time: {img_time:.6f} (Δt={dt*1000:.1f} ms) ===")
        print(f"Angular vel [rad/s]: "
              f"{imu_msg.angular_velocity.x:.3f}, "
              f"{imu_msg.angular_velocity.y:.3f}, "
              f"{imu_msg.angular_velocity.z:.3f}")
        print(f"Linear acc [m/s²]: "
              f"{imu_msg.linear_acceleration.x:.3f}, "
              f"{imu_msg.linear_acceleration.y:.3f}, "
              f"{imu_msg.linear_acceleration.z:.3f}")

        # 영상 보기
        cv2.imshow("Frame", cv_img)
        key = cv2.waitKey(1)
        if key == 27:  # ESC 종료
            break

    bag.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
