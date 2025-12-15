#!/bin/bash
set -e

BASE_DIR="/home/hightorque"
RKLLAMA_DIR="$BASE_DIR/rkllama"

REPO_ID="c01zaut/Qwen2.5-3B-Instruct-RK3588-1.1.4"
MODEL_FILE="Qwen2.5-3B-Instruct-rk3588-w8a8-opt-0-hybrid-ratio-0.5.rkllm"

echo "========================================"
echo " rkllama model pull script"
echo "========================================"
echo " Base dir : $BASE_DIR"
echo " Repo ID  : $REPO_ID"
echo " File     : $MODEL_FILE"
echo "========================================"

# rkllama 디렉토리 확인
if [ ! -d "$RKLLAMA_DIR" ]; then
    echo "[ERROR] rkllama directory not found: $RKLLAMA_DIR"
    echo "        Clone rkllama first."
    exit 1
fi

cd "$RKLLAMA_DIR"

# rkllama 실행 파일 확인
if ! command -v rkllama >/dev/null 2>&1; then
    echo "[ERROR] rkllama command not found."
    echo "        Check build or PATH settings."
    exit 1
fi

echo "[1/1] Pulling model..."
printf "%s\n%s\n" "$REPO_ID" "$MODEL_FILE" | rkllama pull

echo "----------------------------------------"
echo " Model pull completed"
echo "----------------------------------------"
