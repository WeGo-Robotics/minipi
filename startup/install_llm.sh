#!/bin/bash
set -e

BASE_DIR="/home/hightorque"
REPO_URL="https://github.com/NotPunchnox/rkllama.git"
REPO_NAME="rkllama"
BRANCH_NAME="Beta"

MODEL_SCRIPT="install_llm_model.sh"

# serve 관련 설정
SERVE_LOG="/tmp/rkllama_serve.log"
SERVE_CMD="rkllama serve"
WAIT_SECONDS=30

echo "========================================"
echo " rkllama install + serve + model pull"
echo " Base dir: $BASE_DIR"
echo "========================================"

# git 설치 확인
if ! command -v git >/dev/null 2>&1; then
    echo "[ERROR] git is not installed."
    exit 1
fi

# base dir 확인
if [ ! -d "$BASE_DIR" ]; then
    echo "[ERROR] Base directory does not exist: $BASE_DIR"
    exit 1
fi

cd "$BASE_DIR"

# rkllama clone (이미 있으면 스킵)
if [ ! -d "$REPO_NAME" ]; then
    echo "[1/5] Cloning repository..."
    git clone "$REPO_URL"
    cd "$REPO_NAME"
    echo "[2/5] Fetching branches..."
    git fetch origin
    echo "[3/5] Checking out '$BRANCH_NAME' branch..."
    git checkout "$BRANCH_NAME"
    cd "$BASE_DIR"
else
    echo "[INFO] rkllama already exists. Skipping clone."
fi

# rkllama 커맨드 확인
if ! command -v rkllama >/dev/null 2>&1; then
    echo "[ERROR] rkllama command not found. Check build/PATH."
    exit 1
fi

# serve 실행 여부 확인
if pgrep -f "rkllama serve" >/dev/null; then
    echo "[INFO] rkllama serve already running."
else
    echo "[4/5] Starting rkllama serve (background)..."
    nohup $SERVE_CMD > "$SERVE_LOG" 2>&1 &
fi

# serve 준비 대기
echo "[WAIT] Waiting for rkllama serve to be ready (up to ${WAIT_SECONDS}s)..."
READY=0
for i in $(seq 1 $WAIT_SECONDS); do
    if pgrep -f "rkllama serve" >/dev/null; then
        READY=1
        break
    fi
    sleep 1
done

if [ "$READY" -ne 1 ]; then
    echo "[ERROR] rkllama serve did not start properly."
    echo "        Check log: $SERVE_LOG"
    exit 1
fi
echo "[OK] rkllama serve is running."

# 모델 설치 스크립트 실행
if [ ! -f "$MODEL_SCRIPT" ]; then
    echo "[ERROR] Model install script not found: $BASE_DIR/$MODEL_SCRIPT"
    exit 1
fi

echo "[5/5] Running model installation script..."
chmod +x "$MODEL_SCRIPT"
./"$MODEL_SCRIPT"

echo "========================================"
echo " All done: serve running + model installed"
echo " Serve log: $SERVE_LOG"
echo "========================================"
