#!/bin/bash

# ==============================================================================
# [확인된 절대 경로]
TARGET_FILE="/home/hightorque/soccer_ws/ble_wifi_setup.py"
# ==============================================================================

LOGFILE="/home/hightorque/ble_startup.log"
echo "===== BLE WIFI AUTO START START =====" > $LOGFILE
date >> $LOGFILE

# 1. 파일 존재 여부 최종 확인 (안전장치)
if [ ! -f "$TARGET_FILE" ]; then
    echo "🚨 [CRITICAL ERROR] 파일을 찾을 수 없습니다!"
    echo "경로: $TARGET_FILE"
    echo "🚨 [ERROR] File not found: $TARGET_FILE" >> $LOGFILE
    echo "터미널을 닫지 않고 60초간 대기합니다..."
    sleep 60
    exit 1
fi

# 2. 와이파이 연결 상태 확인 함수
check_wifi_connection() {
    # 연결된 '802-11-wireless' 타입이 있는지 확인
    nmcli -t -f TYPE,STATE connection show --active | grep -q "802-11-wireless:activated"
}

# 3. 초기 연결 확인
if check_wifi_connection; then
    echo "✅ [Init] WiFi already connected."
    echo "✅ [Init] WiFi already connected. Exiting." >> $LOGFILE
    exit 0
fi

echo "❌ [Init] WiFi not connected. Starting BLE setup."
echo "❌ [Init] WiFi not connected. Starting BLE setup." >> $LOGFILE

# 4. 블루투스 켜기 & BLE 스크립트 실행
echo "   -> Resetting Bluetooth..."
# 블루투스 재시작 (혹시 모를 오류 방지)
sudo rfkill unblock bluetooth
echo -e "power on\ndiscoverable on\nexit" | bluetoothctl >> $LOGFILE 2>&1

echo "🚀 [BLE] Running BLE script: $TARGET_FILE"
echo "🚀 [BLE] Running BLE script..." >> $LOGFILE

# [핵심] 절대 경로로 실행
sudo python3 "$TARGET_FILE" >> $LOGFILE 2>&1 &
BLE_PID=$!
echo "   -> BLE PID: $BLE_PID" >> $LOGFILE

# 5. 와이파이 연결 대기 (화면에 점 찍으며 대기)
echo "⏳ Waiting for WiFi connection..."
echo "⏳ [Wait] Waiting for WiFi connection..." >> $LOGFILE

while true; do
    if check_wifi_connection; then
        echo ""
        echo "🎉 [Success] WiFi CONNECTED!"
        echo "🎉 [Success] WiFi CONNECTED!" >> $LOGFILE
        
        # BLE 스크립트 정리
        sudo kill $BLE_PID 2>/dev/null
        exit 0
    fi
    # 화면에 점(.)을 찍어서 동작 중임을 보여줌
    echo -n "."
    sleep 3
done
