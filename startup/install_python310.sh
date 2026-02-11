#!/bin/bash

# =================================================================
# Python 3.10.13 설치 자동화 스크립트 (ARM64 환경 최적화)
# =================================================================

PYTHON_VERSION="3.10.13"
PYTHON_DIR="Python-$PYTHON_VERSION"
PYTHON_TGZ="$PYTHON_DIR.tgz"
PYTHON_URL="https://www.python.org/ftp/python/$PYTHON_VERSION/$PYTHON_TGZ"

echo "--- 1. 필수 빌드 도구 설치 ---"
sudo apt update
# build-essential과 다양한 개발 헤더 파일 설치
sudo apt install -y build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget

if [ $? -ne 0 ]; then
    echo "오류: 필수 빌드 도구 설치에 실패했습니다. 네트워크 또는 권한을 확인하세요."
    exit 1
fi

echo "--- 2. Python $PYTHON_VERSION 소스 다운로드 및 압축 해제 ---"
if [ ! -f "$PYTHON_TGZ" ]; then
    wget "$PYTHON_URL"
fi
tar -xf "$PYTHON_TGZ"

if [ $? -ne 0 ]; then
    echo "오류: 소스 파일 다운로드 또는 압축 해제에 실패했습니다."
    exit 1
fi

cd "$PYTHON_DIR"

echo "--- 3. 컴파일 및 설치 (make altinstall 사용) ---"
# --enable-optimizations: 성능 최적화
# --enable-shared: 공유 라이브러리(libpython3.10.so) 생성
./configure --enable-optimizations --enable-shared

# make -j$(nproc) : 모든 CPU 코어 사용하여 컴파일 속도 향상
make -j$(nproc)

# make altinstall : 시스템 기본 Python을 덮어쓰지 않고 설치
sudo make altinstall

if [ $? -ne 0 ]; then
    echo "오류: Python 컴파일 및 설치에 실패했습니다."
    exit 1
fi

echo "--- 4. 공유 라이브러리 경로 설정 및 캐시 업데이트 ---"
# /usr/local/lib 경로를 시스템 동적 링커에게 알려줌
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/python3.10.conf > /dev/null

# 동적 링커 캐시를 업데이트하여 libpython3.10.so.1.0 오류 해결
sudo ldconfig

if [ $? -ne 0 ]; then
    echo "오류: ldconfig 업데이트에 실패했습니다."
    exit 1
fi

echo "--- 5. 설치 완료 확인 ---"
# 설치된 Python 3.10 바이너리가 정상적으로 실행되는지 확인
if python3.10 --version; then
    echo "Python $PYTHON_VERSION 설치 및 환경 설정이 성공적으로 완료되었습니다."
else
    echo "경고: 설치가 완료되었으나 python3.10 실행에 문제가 있을 수 있습니다."
fi

# 설치 디렉토리 정리
cd ..
rm -rf "$PYTHON_DIR"
rm "$PYTHON_TGZ"

echo "설치 소스 파일 정리 완료."