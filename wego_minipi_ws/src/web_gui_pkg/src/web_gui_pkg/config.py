# web_gui_pkg/config.py

import os
from pathlib import Path
import logging
import sys

# ============================================================
# [CRITICAL FIX] Python 3.8 logging 호환성 패치
# ============================================================
# Twisted 라이브러리가 Python 3.8에서 지원하지 않는 'stacklevel' 인자를
# logging 모듈에 전달하여 발생하는 TypeError를 방지합니다.
if sys.version_info < (3, 11):
    _orig_findCaller = logging.Logger.findCaller

    def _patched_findCaller(self, stack_info=False, stacklevel=1):
        # stacklevel 인자가 들어오더라도 무시하고 원래 함수에는 전달하지 않음
        return _orig_findCaller(self, stack_info)

    logging.Logger.findCaller = _patched_findCaller

# ============================================================
# Twisted / roslibpy signal 충돌 방지 설정
# ============================================================
os.environ.setdefault("TWISTED_DISABLE_SIGNAL_HANDLERS", "1")
os.environ.setdefault("ROSLIBPY_NO_SIGNAL_HANDLERS", "1")
os.environ.setdefault("ROSLIBPY_NO_ROS_CORE", "1")

# ============================================================
# 상수 설정
# ============================================================
ROBOT_NAME = "Mini Pi"
ROSBRIDGE_HOST = "0.0.0.0"
ROSBRIDGE_PORT = 9090
MAX_LOG_LINES = 100

HOME_DIR = Path(os.path.expanduser("~"))

# ============================================================
# 워크스페이스 경로 자동 탐지
# ============================================================
current_file_path = Path(__file__).resolve()

try:
    # wego_minipi_ws/src 폴더를 가리킴 (.../src/web_gui_pkg/src/web_gui_pkg/config.py 기준)
    ROS_SRC_DIR = current_file_path.parents[3]

    # wego_minipi_ws/devel/setup.bash 경로 추정
    ROS_WS_ROOT = current_file_path.parents[4]
    SETUP_BASH_PATH = ROS_WS_ROOT / "devel" / "setup.bash"

    print(f"[CONFIG] Detected Source Dir: {ROS_SRC_DIR}")
except IndexError:
    # 경로 추적 실패 시 기본값
    ROS_SRC_DIR = HOME_DIR / "wego_minipi_ws" / "src"
    SETUP_BASH_PATH = HOME_DIR / "wego_minipi_ws" / "devel" / "setup.bash"
    print("[CONFIG] Warning: Path detection failed. Using default paths.")

# ============================================================
# [CRITICAL FIX] ROS 환경 설정 명령어 조합 (경로 덮어쓰기 방지 로직 적용)
# ============================================================

# 1. 각 작업 공간의 핵심 경로 정의 (rospack이 찾을 수 있는 'share' 또는 'src' 경로)
#    - sim2real_master: install/share 사용
#    - wego_minipi_ws: src 사용
#    - realsense_ws: devel/share 또는 src 사용 (여기서는 devel/share/.. 가 더 일반적)
SIM2REAL_SHARE_PATH = f"{HOME_DIR}/sim2real_master/install/share"
WEGO_SHARE_PATH = f"{HOME_DIR}/wego_minipi_ws/devel/share"
REALSENSE_SHARE_PATH = f"{HOME_DIR}/realsense_ws/devel/share"

ROS_SETUP_COMMAND = (
    # 1. 기본 ROS 및 작업 공간 source 실행
    f"source /opt/ros/noetic/setup.bash; "
    f"source {HOME_DIR}/sim2real_master/install/setup.bash; "
    f"source {SETUP_BASH_PATH}; "  # wego_minipi_ws 경로
    f"source {HOME_DIR}/realsense_ws/devel/setup.bash; "
    # 2. 강제 병합: 어떤 source가 덮어썼더라도 핵심 경로를 ROS_PACKAGE_PATH 맨 앞에 추가하여 복구 및 보장
    #    (각각의 패키지가 있는 share/src 폴더를 직접 추가)
    f"export ROS_PACKAGE_PATH={SIM2REAL_SHARE_PATH}:{WEGO_SHARE_PATH}:{REALSENSE_SHARE_PATH}:$ROS_PACKAGE_PATH"
)

# ============================================================
# LLM 모델 경로 (필요시 추가)
# ============================================================
# LLM_MODEL_PATH = "/path/to/your/llm_model"
