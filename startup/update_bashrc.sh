#!/usr/bin/env bash

BASHRC="$HOME/.bashrc"
WS_LINE="source /home/hightorque/wego_minipi_ws/devel/setup.bash"

# 백업 생성
cp "$BASHRC" "$BASHRC.bak_$(date +%Y%m%d_%H%M%S)"

# 기존 ROS_MASTER_URI/ROS_IP 라인 제거
sed -i '/^export ROS_MASTER_URI=/d' "$BASHRC"
sed -i '/^export ROS_IP=/d' "$BASHRC"

# wego_minipi_ws source 줄이 이미 없는 경우에만 추가
if ! grep -Fxq "$WS_LINE" "$BASHRC"; then
    echo "$WS_LINE" >> "$BASHRC"
fi

# 새로운 항목 추가
cat << 'EOF' >> "$BASHRC"

# === ROS network auto-detect ===
export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
export ROS_IP=$(hostname -I | awk '{print $1}')
# ===============================
EOF

echo "[DONE] .bashrc updated."
echo "Backup: $BASHRC.bak_$(date +%Y%m%d_%H%M%S)"
echo "Run 'source ~/.bashrc' to apply changes."
