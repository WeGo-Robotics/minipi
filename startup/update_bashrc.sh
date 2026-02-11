#!/usr/bin/env bash

BASHRC="$HOME/.bashrc"
TEMP_BASHRC_FILE="$HOME/.bashrc.temp"

# --- 1. 백업 생성 ---
BACKUP_FILE="$BASHRC.bak_$(date +%Y%m%d_%H%M%S)"
cp "$BASHRC" "$BACKUP_FILE"
echo "[INFO] Backup created: $BACKUP_FILE"

# --- 2. 기존 ROS 설정 및 wego_minipi_ws source 라인 제거 ---
# wego_minipi_ws source, ROS_MASTER_URI, ROS_IP, wego_minipi_ws alias, echo 라인 제거
sed -i '/^source \/home\/hightorque\/wego_minipi_ws\/devel\/setup.bash/d' "$BASHRC"
sed -i '/^source ~\/wego_minipi_ws\/devel\/setup.bash/d' "$BASHRC"
sed -i '/^export ROS_MASTER_URI=/d' "$BASHRC"
sed -i '/^export ROS_IP=/d' "$BASHRC"
sed -i '/^alias wego_minipi_ws=/d' "$BASHRC"
sed -i '/^echo "========================/,$d' "$BASHRC" # 마지막 echo 블록 제거

# --- 3. ROS 환경 설정 및 병합 로직 삽입 ---

cat << 'EOF' >> "$BASHRC"

# ==========================================================
# ROS Workspace Environment Setup (Auto-merged)
# ==========================================================

# wego_minipi_ws 로드 전에 현재 ROS_PACKAGE_PATH (sim2real_master 포함)를 임시 저장
# (fishros initialize 블록 다음에 위치해야 함)
TEMP_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH

# ROS Network Auto-detect
export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
export ROS_IP=$(hostname -I | awk '{print $1}')

# wego_minipi_ws 환경 로드 (이 시점에 다른 경로가 덮어쓰여질 수 있음)
source /home/hightorque/wego_minipi_ws/devel/setup.bash

# 경로 복원 및 병합: wego_minipi_ws가 덮어쓴 후, 사라진 경로들을 다시 앞에 추가하여 복원
export ROS_PACKAGE_PATH=$TEMP_ROS_PACKAGE_PATH:$ROS_PACKAGE_PATH

# Alias
alias wego_minipi_ws='cd ~/wego_minipi_ws && source devel/setup.bash'

echo "========================"
echo "wego_minipi_ws: source devel"
echo "========================"

# ==========================================================
EOF

echo "[DONE] .bashrc updated with ROS merge logic."
echo "Run 'source ~/.bashrc' to apply changes."