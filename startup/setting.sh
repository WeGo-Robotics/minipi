#!/bin/bash

# 사용자 정의
TARGET_USER="hightorque"
TARGET_GROUP="cat"
TARGET_HOME_DIR="/home/$TARGET_USER"

# 현재 스크립트가 실행되는 디렉토리 (source directory)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "현재 스크립트 디렉토리: $SCRIPT_DIR"

TARGET_DESKTOP_DIR1="$TARGET_HOME_DIR/Desktop"
TARGET_DESKTOP_DIR2="$TARGET_HOME_DIR/.config/autostart"
TARGET_SIM2REAL_MASTER_DIR="$TARGET_HOME_DIR/sim2real_master/install/share/sim2real_master"

# 1. .desktop 파일 복사 및 소유권/실행 권한 설정
echo "--- .desktop 파일 복사 및 권한 설정 시작 ---"
# joy_Switch_alg.desktop
if [ -f "$SCRIPT_DIR/joy_Switch_alg.desktop" ]; then
    sudo cp "$SCRIPT_DIR/joy_Switch_alg.desktop" "$TARGET_DESKTOP_DIR1/"
    # 복사 후 소유권을 원래 사용자에게 넘김 (중요!)
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_DESKTOP_DIR1/joy_Switch_alg.desktop"
fi

# robot_wego.desktop
if [ -f "$SCRIPT_DIR/robot_wego.desktop" ]; then
    sudo cp "$SCRIPT_DIR/robot_wego.desktop" "$TARGET_DESKTOP_DIR1/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_DESKTOP_DIR1/robot_wego.desktop"

    sudo cp "$SCRIPT_DIR/robot_wego.desktop" "$TARGET_DESKTOP_DIR2/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_DESKTOP_DIR2/robot_wego.desktop"
fi

# custom_startup.desktop
if [ -f "$SCRIPT_DIR/custom_startup.desktop" ]; then
    sudo cp "$SCRIPT_DIR/custom_startup.desktop" "$TARGET_DESKTOP_DIR1/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_DESKTOP_DIR1/custom_startup.desktop"

    sudo cp "$SCRIPT_DIR/custom_startup.desktop" "$TARGET_DESKTOP_DIR2/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_DESKTOP_DIR2/custom_startup.desktop"
fi

# 실행 권한 부여 (Desktop 폴더) - 소유권 변경 후 실행해야 root 소유 파일에 대한 문제 해결됨
if [ -d "$TARGET_DESKTOP_DIR1" ]; then
    echo "$TARGET_DESKTOP_DIR1 내 .desktop 파일에 실행 권한을 부여합니다."
    sudo chmod +x "$TARGET_DESKTOP_DIR1"/*.desktop
fi
if [ -d "$TARGET_DESKTOP_DIR2" ]; then
    echo "$TARGET_DESKTOP_DIR2 내 .desktop 파일에 실행 권한을 부여합니다."
    sudo chmod +x "$TARGET_DESKTOP_DIR2"/*.desktop
fi

echo "--- .desktop 파일 복사 및 권한설정 완료 ---"

# 2. YAML/Launch 파일 복사 및 소유권 설정
echo "--- YAML/Launch 파일 복사 및 소유권 설정 시작 ---"

# joy_footstep.yaml
if [ -f "$SCRIPT_DIR/joy_footstep.yaml" ]; then
    sudo cp "$SCRIPT_DIR/joy_footstep.yaml" "$TARGET_SIM2REAL_MASTER_DIR/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_SIM2REAL_MASTER_DIR/joy_footstep.yaml"
fi

# joy_kid.yaml
if [ -f "$SCRIPT_DIR/joy_kid.yaml" ]; then
    sudo cp "$SCRIPT_DIR/joy_kid.yaml" "$TARGET_SIM2REAL_MASTER_DIR/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_SIM2REAL_MASTER_DIR/joy_kid.yaml"
fi

# joy.yaml
if [ -f "$SCRIPT_DIR/joy.yaml" ]; then
    sudo cp "$SCRIPT_DIR/joy.yaml" "$TARGET_SIM2REAL_MASTER_DIR/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_SIM2REAL_MASTER_DIR/joy.yaml"
fi

# joy_teleop_kid.launch
if [ -f "$SCRIPT_DIR/joy_teleop_kid.launch" ]; then
    sudo cp "$SCRIPT_DIR/joy_teleop_kid.launch" "$TARGET_SIM2REAL_MASTER_DIR/launch/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_SIM2REAL_MASTER_DIR/launch/joy_teleop_kid.launch"
fi

# joy_teleop.launch
if [ -f "$SCRIPT_DIR/joy_teleop.launch" ]; then
    sudo cp "$SCRIPT_DIR/joy_teleop.launch" "$TARGET_SIM2REAL_MASTER_DIR/launch/"
    sudo chown "$TARGET_USER":"$TARGET_GROUP" "$TARGET_SIM2REAL_MASTER_DIR/launch/joy_teleop.launch"
fi

echo "--- YAML/Launch 파일 복사 및 소유권 설정 완료 ---"

# 3. startup 폴더 복사 및 스크립트 실행 권한 부여
echo "--- startup 폴더 복사 및 스크립트 권한 설정 시작 ---"

TARGET_STARTUP_DIR="$TARGET_HOME_DIR/startup"

if [ -d "$SCRIPT_DIR/startup" ]; then
    # 복사 시 -p 옵션을 사용하면 권한을 유지할 수 있지만, root 소유 문제를 피하기 위해 chown을 명시적으로 사용
    sudo cp -r "$SCRIPT_DIR/startup" "$TARGET_HOME_DIR/"
    
    # 복사된 폴더의 소유권을 사용자에게 넘김 (재귀적 -R 사용)
    sudo chown -R "$TARGET_USER":"$TARGET_USER" "$TARGET_STARTUP_DIR"
    echo "$TARGET_STARTUP_DIR 폴더 소유권을 $TARGET_USER 에게 부여했습니다."
    
    # startup 폴더 내 모든 .sh 스크립트에 실행 권한 부여
    echo "$TARGET_STARTUP_DIR 내 .sh 파일에 실행 권한을 부여합니다."
    sudo find "$TARGET_STARTUP_DIR" -type f -name "*.sh" -exec chmod +x {} \;
else
    echo "경고: startup 폴더를 찾을 수 없습니다."
fi

echo "--- startup 폴더 복사 및 스크립트 권한 설정 완료 ---"


# 4. wego_minipi_ws 폴더 복사 (원래 로직 유지)
echo "--- wego_minipi_ws 폴더 복사 시작 ---"
if [ -d "$SCRIPT_DIR/../wego_minipi_ws" ]; then
    echo "wego_minipi_ws 폴더를 $TARGET_HOME_DIR 로 복사합니다."
    sudo cp -r "$SCRIPT_DIR/../wego_minipi_ws" "$TARGET_HOME_DIR/"
    
    # 복사된 폴더의 소유권을 사용자에게 넘김 (원래 로직 유지)
    sudo chown -R "$TARGET_USER":"$TARGET_GROUP" "$TARGET_HOME_DIR/wego_minipi_ws"
else
    echo "경고: wego_minipi_ws 폴더를 찾을 수 없습니다."
fi
echo "--- wego_minipi_ws 폴더 복사 완료 ---"

echo "모든 파일 및 폴더 복사 작업이 완료되었습니다."

exit 0