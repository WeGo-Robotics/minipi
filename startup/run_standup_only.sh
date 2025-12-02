#!/bin/bash

# --- 스탠드업 시퀀스 단독 실행 스크립트 (수정됨) ---
# 
# [실행 순서]
# 1. ROS 환경 로드
# 2. joy_teleop.launch를 백그라운드(&)에서 실행하고 PID 저장
# 3. start_standup.py를 현재 터미널에서 실행하여 완료될 때까지 대기 (Blocking)
# 4. 파이썬 스크립트가 종료되면 저장된 PID를 이용해 joy_teleop 프로세스 종료 (Cleanup)

echo "ROS 환경을 로드하고 Standup 시퀀스를 실행합니다."

# 1. ROS 환경 및 워크스페이스 로드 (필수)
source /opt/ros/noetic/setup.bash
source /home/hightorque/soccer_ws/devel/setup.bash

# 2. joy_teleop.launch 실행 (백그라운드)
echo "joy_teleop_kid.launch 를 백그라운드에서 실행합니다."
# &를 사용하여 백그라운드에서 실행하고, PID($!)를 저장합니다.
roslaunch sim2real_master joy_teleop_kid.launch > /dev/null 2>&1 &
JOY_TELEOP_PID=$!
echo "   -> joy_teleop 런치 프로세스 PID: $JOY_TELEOP_PID"

# 3. Standup 스크립트 실행 (Blocking)
# 기존 gnome-terminal 실행을 제거하고 현재 스크립트 흐름에서 실행하여
# 파이썬 스크립트가 완전히 종료될 때까지 **대기(Blocking)** 하도록 합니다.
echo "🏃‍♂️ Standup Sequence Python 스크립트를 현재 창에서 실행합니다. (종료 대기)"
python3 /home/hightorque/soccer_ws/start_standup.py

# 파이썬 스크립트가 종료되면 다음 라인으로 진행합니다.

# 4. joy_teleop.launch 프로세스 종료 (Cleanup)
echo "Standup 스크립트가 완료되었습니다. joy_teleop 런치 프로세스를 종료합니다."

# PID가 유효하고 여전히 실행 중인지 확인
if [ -n "$JOY_TELEOP_PID" ] && ps -p $JOY_TELEOP_PID > /dev/null; then
    # roslaunch 프로세스에 안전한 종료 시그널(SIGINT)을 보냅니다.
    kill -SIGINT $JOY_TELEOP_PID
    echo "   -> PID $JOY_TELEOP_PID (joy_teleop) 종료 명령 전송 완료."
    
    # 프로세스가 정리될 시간을 잠시 대기
    sleep 2 
    
    # 혹시 남아있는 프로세스가 있다면 강제 종료 (선택적)
    if ps -p $JOY_TELEOP_PID > /dev/null; then
        kill -SIGKILL $JOY_TELEOP_PID
        echo "   -> 잔여 프로세스를 강제 종료(SIGKILL)했습니다."
    fi
else
    echo "joy_teleop 런치 프로세스가 이미 종료되었거나 PID를 찾을 수 없어 종료 작업을 생략합니다."
fi

echo "모든 작업 완료 및 정리되었습니다."
exit 0