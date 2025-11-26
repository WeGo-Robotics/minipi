#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge
# 🚨 압축되지 않은 깊이 데이터를 위해 Image 메시지를 사용합니다.
from sensor_msgs.msg import Image, CameraInfo
import cv2 

# CameraInfo에서 가져온 K 행렬로 3D 좌표를 계산하는 함수
def deproject_pixel_to_point(depth_image_pixel, depth_in_meters, K_matrix):
    """
    단일 픽셀 (u, v)과 깊이 값 (Z)을 사용하여 3D 좌표 (X, Y, Z)를 계산합니다.
    """
    # K 행렬에서 초점 거리와 주점 추출
    fx = K_matrix[0, 0]
    fy = K_matrix[1, 1]
    cx = K_matrix[0, 2]
    cy = K_matrix[1, 2]

    u, v = depth_image_pixel
    Z = depth_in_meters
    
    # 역투영 (Deprojection) 공식
    X = Z * (u - cx) / fx
    Y = Z * (v - cy) / fy

    return np.array([X, Y, Z])

class PointCloudCreator:

    def __init__(self):
        rospy.init_node('pointcloud_creator', anonymous=True)
        self.bridge = CvBridge()
        self.K = None
        self.depth_scale = 0.001 # 1mm -> 1m 변환
        
        # 0.5m ~ 4m 사이의 깊이만 시각화에 사용
        self.min_display_depth_mm = 500.0 
        self.max_display_depth_mm = 4000.0 

        # CameraInfo 구독
        self.info_sub = rospy.Subscriber(
            '/camera/depth/camera_info', 
            CameraInfo, 
            self.camera_info_callback, 
            queue_size=1)
        
        # 🚨 깊이 이미지 구독: '/image_rect_raw' 토픽과 Image 메시지 사용
        self.depth_sub = rospy.Subscriber(
            '/camera/depth/image_rect_raw',  
            Image, 
            self.depth_callback,
            queue_size=1)

        rospy.loginfo('ROS 1 포인트 클라우드 및 이미지 시각화 노드 시작. CameraInfo 대기 중...')
        
        # 윈도우 생성
        cv2.namedWindow("Depth Image Visualization", cv2.WINDOW_AUTOSIZE)


    def camera_info_callback(self, msg):
        """CameraInfo 메시지에서 K 행렬을 추출하고 저장합니다."""
        if self.K is None:
            # K 행렬을 3x3 NumPy 행렬로 변환
            self.K = np.array(msg.K).reshape((3, 3))
            rospy.loginfo('CameraInfo (K 행렬) 수신 완료.')
            self.info_sub.unregister()


    def depth_callback(self, msg):
        """깊이 이미지를 수신하고 처리합니다."""
        if self.K is None:
            return

        try:
            # 1. Image 메시지를 16비트 깊이 값(mm)을 담은 NumPy 배열로 변환
            # encoding='passthrough'는 16UC1을 유지합니다.
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 깊이 데이터가 유효하지 않은 경우 처리
            if depth_image.size == 0:
                rospy.logwarn("수신된 깊이 이미지가 비어있습니다.")
                return

            # --- 이미지 시각화 (Depth Colormapping) ---
            
            # 깊이 이미지를 float32로 변환하여 처리
            depth_float = depth_image.astype(np.float32)

            # 1. 시각화 범위로 클리핑
            depth_clipped = np.clip(depth_float, self.min_display_depth_mm, self.max_display_depth_mm)
            
            # 2. 0-255 스케일로 정규화 (최소 ~ 최대 깊이 범위만 사용)
            range_mm = self.max_display_depth_mm - self.min_display_depth_mm
            # (depth - min) / range * 255
            depth_normalized = (depth_clipped - self.min_display_depth_mm) / range_mm * 255
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # 3. 컬러맵 적용
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # 4. 깊이가 0 (측정 실패 또는 너무 가까움)인 영역을 마스크하여 흰색으로 표시
            # 검은색 막대 문제를 해결하기 위한 중요한 단계
            mask_zero = (depth_image == 0)
            depth_colormap[mask_zero] = [255, 255, 255] # 흰색 (BGR)
            
            # 5. 시각화
            # cv2.imshow("Depth Image Visualization", depth_colormap)
            # cv2.waitKey(1)
            
            # --- 포인트 클라우드 계산 ---
            H, W = depth_image.shape
            point_cloud = []
            
            # 6. 모든 픽셀을 순회하며 3D 좌표 계산
            for v in range(H):
                for u in range(W):
                    # 16비트 깊이 값 (mm)
                    depth_value_mm = depth_image[v, u]
                    
                    # 깊이 값이 유효한지 확인 (0 초과)
                    if depth_value_mm > 0.0: 
                        # 미터로 변환
                        depth_value_m = depth_value_mm * self.depth_scale
                        
                        point_3d = deproject_pixel_to_point(
                            (u, v), 
                            depth_value_m, 
                            self.K
                        )
                        point_cloud.append(point_3d)

            point_cloud_np = np.array(point_cloud)
            # 포인트 클라우드 데이터가 성공적으로 계산되었음을 콘솔에 출력
            rospy.loginfo('✅ 총 생성된 포인트 클라우드 수: %d' % len(point_cloud_np))
            if len(point_cloud_np) > 0:
                 rospy.loginfo('첫 5개 포인트 (X, Y, Z):\n %s' % point_cloud_np[:5])
            
            
        except Exception as e:
            rospy.logerr('이미지/포인트 클라우드 처리 중 오류 발생: %s' % str(e))
            # cv2.waitKey(1)

# --- 메인 실행 루프 ---

if __name__ == '__main__':
    try:
        creator = PointCloudCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 노드가 종료될 때 OpenCV 윈도우를 닫습니다.
        cv2.destroyAllWindows()