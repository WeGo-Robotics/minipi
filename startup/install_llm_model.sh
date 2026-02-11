#!/bin/bash
set -e

BASE_DIR="/home/hightorque"
RKLLAMA_DIR="$BASE_DIR/rkllama"
MODEL_DIR="$BASE_DIR/RKLLAMA/models"

REPO_ID="c01zaut/gemma-2-2b-it-rk3588-1.1.1"
MODEL_FILE="gemma-2-2b-it-rk3588-w8a8-opt-1-hybrid-ratio-0.5.rkllm"
MODEL_SRC_DIR="gemma-2-2b-it-rk3588-w8a8-opt-1-hybrid-ratio-0.5"
MODEL_DST_DIR="gemma2:2b"

echo "========================================"
echo " rkllama serve + model pull script"
echo "========================================"
echo " Base dir : $BASE_DIR"
echo " Repo ID  : $REPO_ID"
echo " File     : $MODEL_FILE"
echo "========================================"

# rkllama 디렉토리 확인
if [ ! -d "$RKLLAMA_DIR" ]; then
    echo "[ERROR] rkllama directory not found: $RKLLAMA_DIR"
    exit 1
fi

cd "$RKLLAMA_DIR"

# rkllama 실행 파일 확인
if ! command -v rkllama >/dev/null 2>&1; then
    echo "[ERROR] rkllama command not found."
    exit 1
fi

echo "[1/4] Starting rkllama serve..."
rkllama serve > rkllama_serve.log 2>&1 &
SERVE_PID=$!

echo "       rkllama serve PID: $SERVE_PID"

# 스크립트 종료 시 serve 정리
cleanup() {
    echo "[CLEANUP] Stopping rkllama serve (PID: $SERVE_PID)"
    kill $SERVE_PID >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[2/4] Waiting 30 seconds for server to be ready..."
sleep 30

echo "[3/4] Pulling model..."
printf "%s\n%s\n" "$REPO_ID" "$MODEL_FILE" | rkllama pull
sleep 5

echo "[4/4] Renaming model directory..."

# 모델 디렉토리 확인
if [ ! -d "$MODEL_DIR/$MODEL_SRC_DIR" ]; then
    echo "[ERROR] Model directory not found:"
    echo "        $MODEL_DIR/$MODEL_SRC_DIR"
    exit 1
fi

# 대상 디렉토리 중복 방지
if [ -d "$MODEL_DIR/$MODEL_DST_DIR" ]; then
    echo "[ERROR] Target directory already exists:"
    echo "        $MODEL_DIR/$MODEL_DST_DIR"
    exit 1
fi

cd "$MODEL_DIR"
mv "$MODEL_SRC_DIR" "$MODEL_DST_DIR"

echo "----------------------------------------"
echo " Model pull & rename completed"
echo "   $MODEL_SRC_DIR  →  $MODEL_DST_DIR"
echo "----------------------------------------"
