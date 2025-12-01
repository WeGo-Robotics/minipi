#!/bin/bash

# =============================================================
# [⭐️ 핵심 수정 사항 1] 메인 스크립트에 ROS 환경 로드
# 이 스크립트에서 직접 실행되는 모든 ROS 명령어(python3)를 위해 필수입니다.
source /opt/ros/noetic/setup.bash
source /home/hightorque/soccer_ws/devel/setup.bash 
# =============================================================

# [⭐️ 핵심 수정 사항 2] 네트워크 IP를 메인 스크립트에 강제로 설정
# 노드들이 통신할 자신의 IP를 명확히 지정하여 통신 오류를 방지합니다.
# export ROS_IP=192.168.0.48

export ROS_IP=$(hostname -I | awk '{print $1}')

echo "Set ROS_IP to $ROS_IP"

# --- 1. 이전 로봇 구동 서비스 중지 및 노드 정리 ---
echo "1️⃣ 이전 서비스 중지 및 로봇 노드 정리를 시도합니다."
# 기존 systemctl 중지 시도 (오류가 나도 계속 진행)
systemctl --user stop sim2real_master.service 2>/dev/null || true 
# 실행 중인 로봇 제어 ROS 노드(가정: /sim2real_master)를 강제로 종료합니다.
rosnode kill /sim2real_master 2>/dev/null || true
sleep 2

# --- 2. 로봇 구동 터미널 시작 (백그라운드) ---
echo "2️⃣ 로봇 구동 터미널(sim2real)을 시작합니다."
# [⭐️ 수정] 복잡한 명령을 헬퍼 스크립트로 분리하여 gnome-terminal 명령을 단순화합니다.
# 헬퍼 스크립트를 실행 파일로 만든 후 사용하세요. (chmod +x /home/hightorque/soccer_ws/sim2real_run.sh)
gnome-terminal --window --title="sim2real" -- bash -c "~/Desktop/joy_Switch_alg/joy_switch_alg.sh" &
  
# [⭐️ 핵심 수정 사항 3] sim2real 노드에 충분한 초기화 시간 제공 (20초)
sleep 20 

# --- 3. ROS Bridge 서버 터미널 시작 (백그라운드) ---
echo "3️⃣ ROS Bridge 서버를 시작합니다."
gnome-terminal --window --title="ROS Bridge Server" -- bash -c "source /opt/ros/noetic/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 address:=192.168.0.48; exec bash" &
sleep 5

# --- 4. NiceGUI 클라이언트 터미널 시작 (백그라운드) ---
echo "4️⃣ NiceGUI 클라이언트를 시작합니다."
gnome-terminal --window --title="NiceGUI Client" -- /bin/bash -c "/home/hightorque/soccer_web_gui.sh; exec bash" &
sleep 5

# ------------------------------------------------------------------
# --- 5. Standup 스크립트 실행 (블로킹 실행) ---
echo "5️⃣ Standup 시퀀스를 실행합니다. (이 스크립트가 끝날 때까지 대기합니다.)"
# 이 스크립트는 ROS_IP가 설정된 메인 셸에서 실행됩니다.
gnome-terminal --window --title="standup" -- /bin/bash -c "/home/hightorque/run_standup_only.sh; " &
# Standup 스크립트 실행이 끝난 후 2초 대기
sleep 2 

echo "모든 자동 시작 프로세스가 완료되었습니다."
