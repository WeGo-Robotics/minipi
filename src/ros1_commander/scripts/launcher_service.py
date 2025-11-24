#!/usr/bin/env python

import rospy
import subprocess
import os
import time

# 생성한 서비스 파일 import
from ros1_commander.srv import LaunchCommand, LaunchCommandResponse

# ROS 1에서 서비스 요청을 처리하는 콜백 함수
def handle_launch_command(req):
    """ROS 2에서 받은 런치 파일 실행 요청을 처리합니다."""
    
    # 런치 파일이 포함된 패키지 이름과 파일 이름 (로봇에 맞게 수정하세요)
    # 예시: 'my_robot_pkg' 패키지의 'my_bringup.launch'
    package_name = 'your_robot_pkg' 
    launch_file = req.launch_file_name

    # roslaunch 명령 구성
    cmd = ['roslaunch', package_name, launch_file]
    
    try:
        rospy.loginfo(f"Request received. Launching ROS 1 file: {' '.join(cmd)}")
        
        # 실제 roslaunch 실행 (새로운 프로세스로 분리)
        # subprocess.Popen을 사용하면 명령이 블록되지 않고 백그라운드에서 실행됩니다.
        # 실제 환경에서는 이 프로세스를 추적하고 관리하는 코드를 추가하는 것이 좋습니다.
        subprocess.Popen(cmd) 

        # 즉시 성공 응답
        return LaunchCommandResponse(
            success=True,
            message=f"Successfully started launch command for: {launch_file}"
        )
    except Exception as e:
        rospy.logerr(f"Failed to launch file: {e}")
        return LaunchCommandResponse(
            success=False,
            message=f"Failed to start launch file. Check permissions/path: {e}"
        )

def launcher_service_server():
    rospy.init_node('ros1_launcher_service_server')
    # ROS 2에서 호출할 서비스 이름 설정
    s = rospy.Service('robot/launch_control', LaunchCommand, handle_launch_command)
    rospy.loginfo("--- ROS 1 Launch Control Service Ready (robot/launch_control) ---")
    rospy.spin()

if __name__ == "__main__":
    launcher_service_server()