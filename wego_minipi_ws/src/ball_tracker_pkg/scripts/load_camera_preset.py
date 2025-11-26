#!/usr/bin/env python3
import rospy
import json
import os
import sys

# JSON 파일의 경로 (실제 파일 경로로 수정 필요)
# 예: Camera Tuner가 저장한 파일
DEFAULT_JSON_PATH = os.path.expanduser("~/soccer_ws/cam_preset/fisheye_1029.json") 

def load_params_from_json(json_path):
    """
    JSON 파일에서 카메라 설정을 읽어 ROS 파라미터 서버에 로드합니다.
    """
    if not os.path.exists(json_path):
        rospy.logerr(f"JSON file not found at: {json_path}")
        return False

    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        rospy.logerr(f"Failed to read or parse JSON file: {e}")
        return False

    rospy.init_node('camera_param_loader', anonymous=True)
    
    # ⭐️ ROS 파라미터 서버에 값 설정 ⭐️
    rospy.set_param('/camera_settings/brightness', data.get("BRIGHTNESS"))
    rospy.set_param('/camera_settings/contrast', data.get("CONTRAST"))
    rospy.set_param('/camera_settings/saturation', data.get("SATURATION"))
    rospy.set_param('/camera_settings/hue', data.get("HUE"))
    rospy.set_param('/camera_settings/gain', data.get("GAIN"))
    rospy.set_param('/camera_settings/exposure', data.get("EXPOSURE"))
    rospy.set_param('/camera_settings/white_balance_temperature', data.get("WB_TEMPERATURE"))
    
    # Auto 설정은 0/1 값으로 변환
    rospy.set_param('/camera_settings/auto_exposure', bool(data.get("AUTO_EXPOSURE")))
    rospy.set_param('/camera_settings/auto_white_balance', bool(data.get("AUTO_WB")))
    
    # 해상도/FPS는 개별 노드의 launch 파일에서 직접 사용하는 경우가 많음
    rospy.set_param('/camera_settings/resolution_w', data.get("RES")[0] if data.get("RES") else 640)
    rospy.set_param('/camera_settings/resolution_h', data.get("RES")[1] if data.get("RES") else 480)
    rospy.set_param('/camera_settings/fps', data.get("FPS"))
    rospy.set_param('/camera_settings/device', data.get("DEVICE"))

    rospy.loginfo("Camera properties loaded to ROS Parameter Server under /camera_settings.")
    return True

if __name__ == "__main__":
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = DEFAULT_JSON_PATH
        
    if load_params_from_json(path):
        # 파라미터 로드 후 노드가 종료되어도 파라미터는 서버에 남아있습니다.
        rospy.loginfo("Parameter loading complete.")
    else:
        rospy.logerr("Parameter loading failed.")