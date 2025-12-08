#!/bin/bash

# =================================================================
# [스크립트 2] ROS-Friendly 필수 패키지 설치 (NumPy, OpenCV, PyQt5 포함)
# 목적: 충돌 위험 및 컴파일 지연이 높은 패키지들을 apt를 이용해 설치
# 전제: 이 스크립트를 실행하기 전에 requirements.txt에서 
#       'numpy', 'opencv-python', 'PyQt5' 라인은 반드시 제거되어야 합니다.
# =================================================================

echo "--- 1. 시스템 패키지 목록 업데이트 ---"
sudo apt update

# --- NumPy 설치 ---
echo "--- 2. NumPy 설치 (pip 컴파일 방지) ---"
if ! dpkg -l | grep -q "python3-numpy"; then
    sudo apt install -y python3-numpy
    if [ $? -eq 0 ]; then
        echo "python3-numpy (NumPy) 설치 완료."
    else
        echo "오류: python3-numpy 설치에 실패했습니다."
    fi
else
    echo " python3-numpy (NumPy)는 이미 설치되어 있습니다."
fi


# --- OpenCV 설치 ---
echo "--- 3. OpenCV 설치 (pip 컴파일 및 ROS 충돌 방지) ---"
if ! dpkg -l | grep -q "python3-opencv"; then
    sudo apt install -y python3-opencv
    if [ $? -eq 0 ]; then
        echo "python3-opencv (OpenCV) 설치 완료."
    else
        echo "오류: python3-opencv 설치에 실패했습니다."
    fi
else
    echo "python3-opencv (OpenCV)는 이미 설치되어 있습니다."
fi


# --- PyQt5 설치 ---
echo "--- 4. PyQt5 설치 (pip 컴파일 방지) ---"
if ! dpkg -l | grep -q "python3-pyqt5"; then
    # python3-pyqt5 패키지를 설치하여 C++ 컴파일을 피합니다.
    sudo apt install -y python3-pyqt5
    if [ $? -eq 0 ]; then
        echo " python3-pyqt5 (PyQt5) 설치 완료."
    else
        # 설치 실패 시 소스 컴파일이 필요할 수 있음을 사용자에게 알림
        echo "오류: python3-pyqt5 설치에 실패했습니다. 해당 패키지가 저장소에 없거나 아키텍처를 지원하지 않습니다."
        echo "로컬 GUI 기능이 필수라면, PyQt5를 소스 코드로 직접 컴파일해야 합니다."
    fi
else
    echo " python3-pyqt5 (PyQt5)는 이미 설치되어 있습니다."
fi


echo "--- 5. 필수 패키지 설치 완료 ---"
echo "이제 requirements.txt에서 'numpy', 'opencv-python', 'PyQt5'를 제거/주석 처리한 후, 나머지 패키지를 python3.10 -m pip install로 설치하세요."