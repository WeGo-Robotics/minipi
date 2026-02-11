#!/bin/bash
set -e

BASE_DIR="/home/hightorque"
REPO_URL="https://github.com/NotPunchnox/rkllama.git"
REPO_NAME="rkllama"
BRANCH_NAME="Beta"

MODEL_SCRIPT="install_llm_model.sh"

echo "========================================"
echo " rkllama install + model pull + setup"
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

# -------------------------------------------------
# rkllama clone & branch checkout
# -------------------------------------------------
if [ ! -d "$REPO_NAME" ]; then
    echo "[1/4] Cloning rkllama repository..."
    git clone "$REPO_URL"
    cd "$REPO_NAME"

    echo "[2/4] Fetching branches..."
    git fetch origin

    echo "[3/4] Checking out '$BRANCH_NAME' branch..."
    git checkout "$BRANCH_NAME"

    cd "$BASE_DIR"
else
    echo "[INFO] rkllama already exists. Skipping clone."
fi


# -------------------------------------------------
# setup.sh 실행
# -------------------------------------------------
SETUP_SCRIPT="$BASE_DIR/$REPO_NAME/setup.sh"

if [ ! -f "$SETUP_SCRIPT" ]; then
    echo "[ERROR] setup.sh not found in $BASE_DIR/$REPO_NAME"
    exit 1
fi

echo "----------------------------------------"
echo " Running rkllama setup.sh"
echo "----------------------------------------"

cd "$BASE_DIR/$REPO_NAME"
chmod +x setup.sh
./setup.sh

echo "========================================"
echo " All done:"
echo " - rkllama cloned & configured"
echo " - setup.sh executed"
echo "========================================"
