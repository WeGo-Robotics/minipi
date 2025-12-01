#!/bin/bash

# --- 1. 전역 ROS 환경 설정 (Noetic) ---
# roslaunch/rospy 등 ROS 명령 및 라이브러리 사용을 위해 필요
echo "1️⃣ ROS Noetic 환경 설정을 로드합니다..."
source /opt/ros/noetic/setup.bash

# --- 2. ROS 워크스페이스 환경 설정 및 이동 ---
WS_DIR="/home/hightorque/soccer_ws"

# 워크스페이스 디렉토리로 이동 시도
echo "2️⃣ 워크스페이스 디렉토리로 이동합니다: ${WS_DIR}"
cd "${WS_DIR}" 

# cd 명령 실패 시 오류를 출력하고 종료
if [ $? -ne 0 ]; then
    echo "❌ 오류: ${WS_DIR} 디렉토리를 찾을 수 없습니다. 경로를 확인하세요."
    # 오류 메시지를 사용자에게 보여주기 위해 쉘 유지
    exec bash
    exit 1
fi

# 워크스페이스 환경 변수 로드 (python3.10 경로 및 패키지 설정)
source devel/setup.bash

# --- 3. NiceGUI 애플리케이션 실행 ---
APP_SCRIPT="src/niceGUI_ROS_funcCom.py"
PYTHON_EXEC="python3.10"

echo "3️⃣ NiceGUI 애플리케이션 실행을 시도합니다: ${PYTHON_EXEC} ${APP_SCRIPT}"
# NiceGUI 실행 시도
"${PYTHON_EXEC}" "${APP_SCRIPT}"
STATUS=$?

echo "---"
# Python 실행 결과에 따라 메시지 출력
if [ $STATUS -ne 0 ]; then
    echo "❌ NiceGUI 실행이 실패했습니다 (종료 코드: $STATUS)."
    echo "    - ⚠️ 원인 확인: ${APP_SCRIPT} 파일이 ${WS_DIR}/src/ 안에 있는지 확인하세요."
    echo "    - ⚠️ 원인 확인: 'pip install nicegui' 등 필요한 라이브러리가 설치되었는지 확인하세요."
else
    echo "✅ NiceGUI 애플리케이션이 종료되었습니다."
fi

# 스크립트 실행이 끝난 후 터미널 창이 바로 닫히지 않고 유지되도록 합니다.
echo "Press Enter to close this terminal..."
read -r
