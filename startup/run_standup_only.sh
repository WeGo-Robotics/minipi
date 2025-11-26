#!/bin/bash

# --- 스탠드업 시퀀스 단독 실행 스크립트 ---
# 
# [실행 전제 조건]
# 1. sim2real 로봇 구동 노드가 이미 켜져 있어야 합니다.
# 2. ROS Bridge 서버가 이미 켜져 있어야 합니다.
# 
# 이 스크립트는 새로운 터미널 창을 열어 파이썬 코드를 실행하고,
# 파이썬 코드가 완료되면 (rospy.signal_shutdown 호출 시) 터미널 창이 닫힙니다.

echo "⭐️ ROS 환경을 로드하고 Standup 시퀀스를 실행합니다."

# 1. ROS 환경 및 워크스페이스 로드 (필수)
source /opt/ros/noetic/setup.bash
source /home/hightorque/soccer_ws/devel/setup.bash

# 2. Standup 스크립트를 새로운 터미널 창에서 실행
# exec bash를 제거하여 파이썬 스크립트 종료 시 터미널 창이 자동으로 닫히도록 설정합니다.
gnome-terminal --window --title="Standup Sequence Runner" -- bash -c "
  python3 /home/hightorque/soccer_ws/start_standup.py
"
