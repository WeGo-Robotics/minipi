#!/bin/bash

# 1. 와이파이 체크 및 BLE 설정 (이 스크립트는 와이파이 연결 전까지 안 끝남)
/home/hightorque/startup/default/wifi_check.sh

# ---------------------------------------------------------
# 위 스크립트가 exit 0을 하고 넘어왔다는 것은 와이파이가 연결되었다는 뜻입니다.
# ---------------------------------------------------------

# [추가 팁] 와이파이가 새로 연결되었으므로 IP가 바뀌었을 수 있습니다.
# IP를 동적으로 가져와서 환경변수로 넘겨주는 것이 더 안전합니다.
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "Current WiFi IP: $CURRENT_IP"

# 2. 기존 로봇 구동 스크립트 실행
# 기존 스크립트에서 export ROS_IP=192.168.0.48 부분을 
# export ROS_IP=$1 로 받도록 수정하면 더욱 좋습니다. 
# (여기서는 기존 파일 그대로 실행한다고 가정합니다)

/home/hightorque/startup/default/master_autostart.sh