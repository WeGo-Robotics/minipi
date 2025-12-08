#!/bin/bash

# --- 스크립트 설명 ---
# 이 스크립트는 현재 디렉토리(startup 폴더)에 있는 특정 파일들과 폴더를 
# 사용자가 지정한 홈 디렉토리 내의 위치로 복사합니다.

# 현재 스크립트가 실행되는 디렉토리 (source directory)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "현재 스크립트 디렉토리: $SCRIPT_DIR"

# 대상 디렉토리 설정
TARGET_HOME_DIR="/home/hightorque"

TARGET_DESKTOP_DIR="$TARGET_HOME_DIR/Desktop"
TARGET_DESKTOP_DIR2="$TARGET_HOME_DIR/.config/autostart"

TARGET_SIM2REAL_MASTER_DIR="$TARGET_HOME_DIR/sim2real_master/install/share/sim2real_master"

# 1. .desktop 파일들을 ~/Desktop으로 복사 (여기서는 /home/hightorque/Desktop)
echo "--- .desktop 파일 복사 시작 ---"
# joy_Switch_alg.desktop
if [ -f "$SCRIPT_DIR/joy_Switch_alg.desktop" ]; then
    echo "joy_Switch_alg.desktop 파일을 $TARGET_DESKTOP_DIR1 로 복사합니다."
    cp "$SCRIPT_DIR/joy_Switch_alg.desktop" "$TARGET_DESKTOP_DIR1/"
else
    echo "경고: joy_Switch_alg.desktop 파일을 찾을 수 없습니다."
fi

# robot_wego.desktop
if [ -f "$SCRIPT_DIR/robot_wego.desktop" ]; then
    echo "robot_wego.desktop 파일을 $TARGET_DESKTOP_DIR1 로 복사합니다."
    cp "$SCRIPT_DIR/robot_wego.desktop" "$TARGET_DESKTOP_DIR1/"

    echo "robot_wego.desktop 파일을 $TARGET_DESKTOP_DIR2 로 복사합니다."
    cp "$SCRIPT_DIR/robot_wego.desktop" "$TARGET_DESKTOP_DIR2/"
else
    echo "경고: robot_wego.desktop 파일을 찾을 수 없습니다."
fi
echo "--- .desktop 파일 복사 완료 ---"

# 2. .sh 스크립트 파일들과 wego_minipi_ws 폴더를 /home/hightorque로 복사
echo "--- .sh 스크립트 파일 및 폴더 복사 시작 ---"

if [ -d "$SCRIPT_DIR/startup" ]; then
    echo "startup 폴더를 $TARGET_HOME_DIR 로 복사합니다."
    cp -r "$SCRIPT_DIR/startup" "$TARGET_HOME_DIR/"
else
    echo "경고: startup 폴더를 찾을 수 없습니다."
fi

# joy_footstep.yaml
if [ -f "$SCRIPT_DIR/joy_footstep.yaml" ]; then
    echo "joy_footstep.yaml 파일을 $TARGET_SIM2REAL_MASTER_DIR 로 복사합니다."
    cp "$SCRIPT_DIR/joy_footstep.yaml" "$TARGET_SIM2REAL_MASTER_DIR/"
else
    echo "경고: joy_footstep.yaml 파일을 찾을 수 없습니다."
fi

# joy_kid.yaml
if [ -f "$SCRIPT_DIR/joy_kid.yaml" ]; then
    echo "joy_kid.yaml 파일을 $TARGET_SIM2REAL_MASTER_DIR 로 복사합니다."
    cp "$SCRIPT_DIR/joy_kid.yaml" "$TARGET_SIM2REAL_MASTER_DIR/"
else
    echo "경고: joy_kid.yaml 파일을 찾을 수 없습니다."
fi

# joy_teleop_kid.launch
if [ -f "$SCRIPT_DIR/joy_teleop_kid.launch" ]; then
    echo "joy_teleop_kid.launch 파일을 $TARGET_SIM2REAL_MASTER_DIR/launch 로 복사합니다."
    cp "$SCRIPT_DIR/joy_teleop_kid.launch" "$TARGET_SIM2REAL_MASTER_DIR/launch/"
else
    echo "경고: joy_teleop_kid.launch 파일을 찾을 수 없습니다."
fi

# joy_teleop.launch
if [ -f "$SCRIPT_DIR/joy_teleop.launch" ]; then
    echo "joy_teleop.launch 파일을 $TARGET_SIM2REAL_MASTER_DIR/launch 로 복사합니다."
    cp "$SCRIPT_DIR/joy_teleop.launch" "$TARGET_SIM2REAL_MASTER_DIR/launch/"
else
    echo "경고: joy_teleop.launch 파일을 찾을 수 없습니다."
fi

# 🌟 wego_minipi_ws 폴더 복사 추가 (재귀적 옵션 -r 사용) 🌟
if [ -d "$SCRIPT_DIR/wego_minipi_ws" ]; then
    echo "wego_minipi_ws 폴더를 $TARGET_HOME_DIR 로 복사합니다."
    # -r 옵션은 디렉토리와 그 내용을 재귀적으로 복사하는 데 사용됩니다.
    cp -r "$SCRIPT_DIR/wego_minipi_ws" "$TARGET_HOME_DIR/"
else
    echo "경고: wego_minipi_ws 폴더를 찾을 수 없습니다."
fi

echo "--- .sh 스크립트 파일 및 폴더 복사 완료 ---"

echo "모든 파일 및 폴더 복사 작업이 완료되었습니다."

exit 0