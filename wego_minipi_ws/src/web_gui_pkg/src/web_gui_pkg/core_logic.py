#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# web_gui_pkg/core_logic.py

import os
import sys
import subprocess
import signal
import threading
import asyncio
import base64
import time
from typing import Dict, Any, Optional, List, Set
from pathlib import Path
import glob
import logging

# NiceGUI
from nicegui import ui, app

# ============================================================
# 설정 파일 임포트 (실패 시 안전장치 포함)
# ============================================================
try:
    from .config import ROBOT_NAME, ROSBRIDGE_HOST, ROSBRIDGE_PORT, MAX_LOG_LINES, ROS_SETUP_COMMAND, ROS_SRC_DIR
except ImportError as e:
    print(f"WARNING: Could not import config.py ({e}). Using Hardcoded Defaults.")

    ROBOT_NAME = "Mini Pi"
    ROSBRIDGE_HOST = "0.0.0.0"
    ROSBRIDGE_PORT = 9090
    MAX_LOG_LINES = 100

    HOME_DIR = Path(os.path.expanduser("~"))
    ROS_SRC_DIR = HOME_DIR / "wego_minipi_ws" / "src"

    # 안전한 source 구현
    def safe_source(path):
        return f'[ -f "{path}" ] && source "{path}"'

    ROS_SETUP_COMMAND = (
        f"source /opt/ros/noetic/setup.bash; "
        f"{safe_source(HOME_DIR / 'realsense_ws/devel/setup.bash')}; "
        f"{safe_source(HOME_DIR / 'wego_minipi_ws/devel/setup.bash')}"
    )

# ============================================================
# Workspace / Package Store
# ============================================================
STORE_PATH = Path.home() / ".wego_launch_manager" / "minipi_workspaces.json"
STORE_PATH.parent.mkdir(parents=True, exist_ok=True)

DEFAULT_STORE = {
    "workspaces": [
        {
            "path": str(ROS_SRC_DIR.parent),  # ~/wego_minipi_ws
            "role": "system",
        }
    ],
    "pinned_packages": [],
    "hidden_packages": [],
}


# ============================================================
# 라이브러리 로드
# ============================================================
try:
    import roslibpy
    import cv2
    import numpy as np
except ImportError as e:
    print(f"FATAL: Required libraries not installed: {e}")
    roslibpy = None
    cv2 = None
    np = None

# OpenCV 속성 매핑 딕셔너리
if cv2 is not None:
    CV_PROPS = {
        "BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
        "CONTRAST": cv2.CAP_PROP_CONTRAST,
        "SATURATION": cv2.CAP_PROP_SATURATION,
        "HUE": cv2.CAP_PROP_HUE,
        "GAIN": cv2.CAP_PROP_GAIN,
        "EXPOSURE": cv2.CAP_PROP_EXPOSURE,
        "WB_TEMPERATURE": cv2.CAP_PROP_WB_TEMPERATURE,
        "AUTO_EXPOSURE": getattr(cv2, "CAP_PROP_AUTO_EXPOSURE", 21),
        "AUTO_WB": getattr(cv2, "CAP_PROP_AUTO_WB", 45),
    }
else:
    CV_PROPS = {}


# ============================================================
# MJPEG Streamer Class
# ============================================================
class MjpegStreamer:
    def __init__(self):
        self._latest_jpeg_bytes: Optional[bytes] = None
        self._lock = threading.Lock()
        self.is_streaming = False

    def update_frame(self, jpeg_bytes: bytes):
        with self._lock:
            self._latest_jpeg_bytes = jpeg_bytes

    def get_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg_bytes

    async def stream_frames(self):
        self.is_streaming = True
        mjpeg_header = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

        async def frame_generator():
            while self.is_streaming:
                frame = self.get_frame()
                if frame:
                    yield mjpeg_header + frame + b"\r\n"
                await asyncio.sleep(0.033)

        from fastapi.responses import StreamingResponse

        return StreamingResponse(
            frame_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    def stop_streaming(self):
        self.is_streaming = False


mjpeg_streamer = MjpegStreamer()


# ============================================================
# Main Core Logic Class
# ============================================================
class NiceGUIRos:
    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.launch_files_data: Dict[str, Any] = {}
        self._launch_files_loaded = False
        self._initialized = False

        # UI References
        self.launch_list_container: Optional[ui.column] = None
        self.search_status_label: Optional[ui.label] = None
        self.logs_container: Optional[ui.label] = None
        self.image_topic_select: Optional[ui.select] = None
        self.image_display: Optional[ui.image] = None
        self.last_frame_label: Optional[ui.label] = None

        # ROS Client
        self.ros_client: Optional[roslibpy.Ros] = None
        self.ros_connected = False
        self._last_ros_connected = False

        # Logs
        # self.current_log_process: Optional[subprocess.Popen] = None
        # self.log_buffer: List[str] = []
        # self.log_executor_task: Optional[asyncio.Task] = None
        self.log_dir = Path.home() / ".wego_launch_logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_views = {}

        # Image
        self.current_image_topic: Optional[roslibpy.Topic] = None
        self.available_image_topics: Dict[str, str] = {}
        self.current_image_src = "data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs="
        self.streamer: MjpegStreamer = mjpeg_streamer
        self.last_frame_time: Optional[float] = None
        self.is_image_subscribed = False
        self.needs_label_update = False

        # LLM Chat
        self.llm_pub: Optional[roslibpy.Topic] = None
        self.llm_sub: Optional[roslibpy.Topic] = None
        self.is_waiting_for_llm = False
        self.loading_spinner_text = "..."
        self.chat_log_buffer: List[Dict[str, str]] = []
        self.chat_log_container: Optional[ui.html] = None
        self.chat_input: Optional[ui.input] = None
        self.send_button: Optional[ui.button] = None
        self.is_llm_streaming = False
        self.current_llm_stream_text = ""

        # Cam Setting
        self.local_cam = None
        self.is_local_cam_running = False
        self.local_cam_task = None

        # Main Loop Capture
        self.main_loop = None
        app.on_startup(self._capture_loop)
        self.on_launch_started = None

    async def _capture_loop(self):
        try:
            self.main_loop = asyncio.get_running_loop()
        except RuntimeError:
            pass

    def on_connection(self, *args):
        print("[ROS] CONNECTED")
        self.ros_connected = True
        self.update_topic_list()
        self.setup_llm_ros_topics()

    def on_close(self, *args):
        print("[ROS] DISCONNECTED")
        self.ros_connected = False
        self.llm_pub = None
        if self.llm_sub:
            try:
                self.llm_sub.unsubscribe()
            except:
                pass
            self.llm_sub = None

    def setup_ros_client(self):
        if not roslibpy or self.ros_client:
            return
        try:
            self.ros_client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
            self.ros_client.on_ready(self.on_connection)

            if hasattr(self.ros_client, "on_close"):
                self.ros_client.on_close(self.on_close)
            else:
                try:
                    self.ros_client.on("close", self.on_close)
                except:
                    pass

            threading.Thread(target=self.ros_client.run_forever, daemon=True).start()
        except Exception as e:
            print("ROS Init Error:", e)

    def initialize(self):
        if not self._initialized:
            self._initialized = True
            self.setup_ros_client()
            ui.timer(0.5, self.poll_ros_status)
            # 주기적 갱신 (5초) - 너무 자주하면 부하 발생
            ui.timer(5.0, self.update_topic_list)
            ui.timer(0.5, self.update_ui_from_thread)

        if self._launch_files_loaded:
            self.render_launch_list()
            if self.search_status_label:
                self.search_status_label.set_text("Launch files cached ✔️")
                self.search_status_label.visible = True
        else:
            asyncio.create_task(self.start_launch_search())

    def poll_ros_status(self):
        connected = bool(self.ros_client and self.ros_client.is_connected)
        if connected and not self._last_ros_connected:
            ui.notify("ROS Connected 🟢", type="positive")
        if (not connected) and self._last_ros_connected:
            ui.notify("ROS Disconnected 🔴", type="warning")
        self._last_ros_connected = connected
        self.ros_connected = connected

    # ============================================================
    # Workspace Search
    # ============================================================
    def _load_store(self) -> dict:
        if not STORE_PATH.exists():
            self._save_store(DEFAULT_STORE)
            return DEFAULT_STORE.copy()

        try:
            import json

            with open(STORE_PATH, "r") as f:
                store = json.load(f)
        except Exception as e:
            print("[STORE] load failed:", e)
            return DEFAULT_STORE.copy()

        # ---- normalize ----
        store.setdefault("workspaces", [])
        store.setdefault("pinned_packages", [])
        store.setdefault("hidden_packages", [])

        for ws in store["workspaces"]:
            ws.setdefault("role", "user")

        return store

    def _save_store(self, data: dict):
        try:
            import json

            with open(STORE_PATH, "w") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print("[STORE] save failed:", e)

    def load_workspaces(self) -> List[dict]:
        store = self._load_store()
        return store.get("workspaces", [])

    def add_workspace(self, path: Path):
        path = path.expanduser().resolve()

        if not path.exists():
            ui.notify(f"경로 없음: {path}", type="negative")
            return

        store = self._load_store()
        ws = store.get("workspaces", [])

        if any(Path(w["path"]) == path for w in ws):
            ui.notify("이미 등록된 워크스페이스입니다", type="warning")
            return

        ws.append(
            {
                "path": str(path),
                "role": "user",
            }
        )
        store["workspaces"] = ws
        self._save_store(store)

        ui.notify(f"워크스페이스 추가됨: {path}", type="positive")

    def set_package_hidden(self, pkg: str, value: bool):
        store = self._load_store()
        s = set(store.get("hidden_packages", []))
        if value:
            s.add(pkg)
        else:
            s.discard(pkg)
        store["hidden_packages"] = sorted(s)
        self._save_store(store)

        self._launch_files_loaded = False
        if not getattr(self, "_reload_task_running", False):
            self._reload_task_running = True
            asyncio.create_task(self._reload_wrapper())

    async def _reload_wrapper(self):
        try:
            await self.start_launch_search()
        finally:
            self._reload_task_running = False

    def set_package_pinned(self, pkg: str, value: bool):
        store = self._load_store()
        s = set(store.get("pinned_packages", []))
        if value:
            s.add(pkg)
        else:
            s.discard(pkg)
        store["pinned_packages"] = sorted(s)
        self._save_store(store)

        self._launch_files_loaded = False
        asyncio.create_task(self.start_launch_search())

    def set_workspace_role(self, ws_path: str, new_role: str):
        print("[DEBUG] set_workspace_role", ws_path, new_role)
        if new_role not in ("system", "external", "user"):
            return

        store = self._load_store()
        changed = False

        for ws in store.get("workspaces", []):
            if ws["path"] == ws_path:
                if ws.get("role") == "system" and new_role != "system":
                    ui.notify("system 워크스페이스는 role 변경 불가", type="warning")
                    return

                ws["role"] = new_role
                changed = True
                break

        if not changed:
            return

        self._save_store(store)

        self._launch_files_loaded = False
        asyncio.create_task(self.start_launch_search())

        ui.notify(f"워크스페이스 role 변경됨 → {new_role}", type="positive")

    def remove_workspace(self, ws_path):
        ws_path = str(ws_path)

        store = self._load_store()
        new_list = []

        removed = False
        for ws in store.get("workspaces", []):
            if ws["path"] == ws_path:
                if ws.get("role") != "user":
                    ui.notify("user 워크스페이스만 삭제할 수 있습니다", type="warning")
                    return
                removed = True
                continue
            new_list.append(ws)

        if not removed:
            ui.notify("워크스페이스를 찾을 수 없습니다", type="warning")
            return

        store["workspaces"] = new_list
        self._save_store(store)

        ui.notify("워크스페이스가 삭제되었습니다", type="positive")

    def _scan_packages_recursive(self, src: Path, max_depth: int = 4) -> Dict[str, Path]:
        """src 아래에서 package.xml을 재귀적으로 찾아 패키지 디렉토리들을 반환"""
        found: Dict[str, Path] = {}

        # depth 제한을 걸고 싶으면 rglob 대신 수동 depth 체크도 가능하지만,
        # 실무에선 max_depth 4 정도면 충분히 안전함.
        for pkg_xml in src.rglob("package.xml"):
            try:
                pkg_dir = pkg_xml.parent
                # src 기준 depth 계산
                rel_parts = pkg_dir.relative_to(src).parts
                if len(rel_parts) > max_depth:
                    continue
                found[pkg_dir.name] = pkg_dir
            except Exception:
                continue

        return found

    def get_workspace_packages(self) -> List[str]:
        store = self._load_store()

        hidden = set(store.get("hidden_packages", []))
        pinned = set(store.get("pinned_packages", []))

        found: Dict[str, Path] = {}

        # -------------------------------------------------
        # 1. workspace별 패키지 수집 (role 규칙 적용)
        # -------------------------------------------------
        for ws in store.get("workspaces", []):
            role = ws.get("role", "user")
            ws_path = Path(ws["path"])
            src = ws_path / "src"

            if not src.exists():
                continue

            scanned = self._scan_packages_recursive(src, max_depth=4)

            for pkg_name, pkg_path in scanned.items():

                # USER workspace → hidden만 제외
                if role == "user":
                    if pkg_name in hidden:
                        continue
                    found[pkg_name] = pkg_path

                # SYSTEM / EXTERNAL → pinned만 허용
                else:
                    if pkg_name not in pinned:
                        continue
                    found[pkg_name] = pkg_path

        # -------------------------------------------------
        # 2. 정렬: pinned 먼저, 그 다음 나머지
        # -------------------------------------------------
        ordered: List[str] = []

        # pinned 우선
        for p in pinned:
            if p in found:
                ordered.append(p)

        # 나머지 (user workspace에서만 의미 있음)
        for p in sorted(found.keys()):
            if p not in ordered:
                ordered.append(p)

        return ordered

    def get_running_launches(self) -> Set[str]:
        """현재 OS에서 실행 중인 roslaunch 목록을 .launch 파일명 기준으로 반환"""
        try:
            cmd = "ps aux | grep roslaunch | grep -v grep"
            result = subprocess.check_output(cmd, shell=True).decode()

            running = set()
            for line in result.splitlines():
                parts = line.split()
                # roslaunch pkg file.launch 형태가 마지막 인자로 등장함
                for token in parts:
                    if token.endswith(".launch"):
                        running.add(token)
            return running
        except Exception as e:
            print("Failed to get running launches:", e)
            return set()

    def find_launch_files(self, packages: List[str]) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for pkg in packages:
            cmd = f"{ROS_SETUP_COMMAND} && rospack find {pkg}"
            try:
                o = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=3)
                path = o.stdout.strip()
                if not path or not os.path.isdir(path):
                    if pkg != "usb_cam":
                        continue

                launch_path = os.path.join(path, "launch")
                files = []
                if os.path.isdir(launch_path):
                    files = [f for f in os.listdir(launch_path) if f.endswith((".launch", ".xml", ".py"))]
                if pkg == "usb_cam" and "usb_cam-test.launch" not in files:
                    files.append("usb_cam-test.launch")

                if files:
                    existing_files = self.launch_files_data.get(pkg, {}).get("files", {})
                    file_data = {}
                    for f in files:
                        if f in existing_files:
                            file_data[f] = existing_files[f]
                        else:
                            file_data[f] = {"status": "STOPPED", "process": None, "button": None}
                    result[pkg] = {"path": path, "files": file_data}
            except Exception as e:
                print(f"Error finding launch files for {pkg}: {e}")
        return result

    async def start_launch_search(self):
        if self.search_status_label:
            self.search_status_label.set_text("Scanning workspace... 🔎")
            self.search_status_label.visible = True
        loop = asyncio.get_event_loop()
        pkgs = await loop.run_in_executor(None, self.get_workspace_packages)
        self.launch_files_data = await loop.run_in_executor(None, self.find_launch_files, pkgs)
        self._launch_files_loaded = True
        self.render_launch_list()
        if self.search_status_label:
            self.search_status_label.set_text(f"Found {len(self.launch_files_data)} packages ✔️")

    def get_available_video_devices(self) -> List[str]:
        try:
            return [d for d in sorted(glob.glob("/dev/video*")) if d[-1].isdigit()]
        except:
            return ["/dev/video0"]

    # ============================================================
    # Launch Control
    # ============================================================
    def toggle_launch(self, pkg: str, file_name: str):
        if pkg not in self.launch_files_data or file_name not in self.launch_files_data[pkg]["files"]:
            return
        info = self.launch_files_data[pkg]["files"][file_name]
        if pkg == "usb_cam" and file_name == "usb_cam-test.launch":
            ui.notify("Use the settings panel for USB CAM.", type="warning")
            return
        if info["status"] == "STOPPED":
            self.run_launch(pkg, file_name, info)
        else:
            self.stop_launch(pkg, file_name, info)

    def toggle_usb_cam_launch(self, video_dev: str, pixel_fmt: str):
        pkg, file = "usb_cam", "usb_cam-test.launch"
        info = self.launch_files_data.get(pkg, {}).get("files", {}).get(file)
        if not info:
            return
        if info["status"] == "STOPPED":
            self.run_launch(pkg, file, info, args=[f"video_device:={video_dev}", f"pixel_format:={pixel_fmt}"])
        else:
            self.stop_launch(pkg, file, info)

    def run_launch(self, pkg: str, file_name: str, info: Dict[str, Any], args: List[str] = None):
        try:
            log_path = self.log_dir / f"{pkg}_{file_name.replace('.launch','').replace('.xml','')}.log"

            launch_cmd = f"roslaunch {pkg} {file_name}"
            if args:
                launch_cmd += " " + " ".join(args)

            cmd = f"{ROS_SETUP_COMMAND} && " f'{launch_cmd} >> "{log_path}" 2>&1'

            wrapper = f'nohup setsid bash -c "{cmd}" >/dev/null 2>&1 & echo $!'

            p = subprocess.Popen(
                wrapper,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            out, _ = p.communicate(timeout=3)
            pid = int(out.decode().strip())

            info.update(
                {
                    "status": "RUNNING",
                    "pid": pid,
                    "pgid": pid,
                    "log_path": str(log_path),
                    "owned": True,
                }
            )

            if info.get("button"):
                info["button"].set_text("RUNNING")
                info["button"].props("color=red icon=pause")

            self.create_log_view(pkg, file_name, str(log_path))
            ui.notify(f"{file_name} started (PID={pid})", type="positive")

            if self.on_launch_started:
                self.on_launch_started(pkg, file_name)

        except Exception as e:
            ui.notify(f"Launch failed: {e}", type="negative")

    def stop_launch(self, pkg: str, file_name: str, info: Dict[str, Any]):
        pid = info.get("pid")
        pgid = info.get("pgid")

        if pid:
            try:
                os.killpg(pgid or pid, signal.SIGTERM)
            except Exception as e:
                ui.notify(f"Stop failed: {e}", type="negative")

        info.update(
            {
                "status": "STOPPED",
                "pid": None,
                "pgid": None,
                "owned": False,
            }
        )

        if info.get("button"):
            info["button"].set_text("RUN")
            info["button"].props("color=green icon=play_arrow")

        self.remove_log_view(pkg, file_name)

    def stop_all_launches(self):
        for pkg, pkg_data in self.launch_files_data.items():
            for file_name, info in pkg_data["files"].items():
                if info["status"] == "RUNNING":
                    self.stop_launch(pkg, file_name, info)
        self.render_launch_list()

    def get_store_snapshot(self) -> dict:
        return self._load_store()

    def get_workspaces(self) -> List[dict]:
        return self._load_store().get("workspaces", [])

    def create_log_view(self, pkg: str, file: str, log_path: str):
        if not self.logs_container:
            return

        key = (pkg, file)
        if key in self.log_views:
            return

        with self.logs_container:
            with ui.expansion(f"{pkg} / {file}", value=True) as exp:
                label = ui.label("").classes("whitespace-pre-wrap font-mono text-xs bg-black text-green-400 p-3 rounded h-80 overflow-auto")

        async def tail():
            proc = await asyncio.create_subprocess_exec("tail", "-F", log_path, stdout=asyncio.subprocess.PIPE)
            buf = []
            while True:
                line = await proc.stdout.readline()
                if not line:
                    await asyncio.sleep(0.1)
                    continue
                buf.append(line.decode(errors="replace").rstrip())
                buf[:] = buf[-MAX_LOG_LINES:]
                label.set_text("\n".join(buf))

        task = asyncio.create_task(tail())
        self.log_views[key] = {"task": task, "exp": exp}

    def remove_log_view(self, pkg: str, file: str):
        key = (pkg, file)
        view = self.log_views.pop(key, None)
        if not view:
            return
        try:
            view["task"].cancel()
            view["exp"].delete()
        except Exception:
            pass

    # ============================================================
    # Render Logic
    # ============================================================
    # core_logic.py 내부의 메서드

    def render_launch_list(self):
        # 컨테이너가 없으면(초기화 안됨) 무시
        if not self.launch_list_container:
            return

        running_set = self.get_running_launches()

        for pkg, pkg_data in self.launch_files_data.items():
            for file_name, info in pkg_data["files"].items():
                if file_name in running_set:
                    info["status"] = "RUNNING"
                else:
                    info["status"] = "STOPPED"

        try:
            # 1. 컨테이너 비우기 (여기서 에러가 가장 많이 발생)
            self.launch_list_container.clear()

            # 2. 내용 채우기
            with self.launch_list_container:
                if self.search_status_label:
                    self.search_status_label.visible = False

                # launch 파일 데이터가 없으면 안내 표시
                if not self.launch_files_data:
                    ui.label("No launch files found.").classes("text-center mt-4")
                    return

                with ui.column().classes("w-full max-w-5xl mx-auto space-y-3"):
                    for pkg, pkg_data in sorted(self.launch_files_data.items()):
                        # usb_cam은 별도 렌더링
                        if pkg == "usb_cam" and "usb_cam-test.launch" in pkg_data["files"]:
                            self.render_usb_cam_controller(pkg, "usb_cam-test.launch", self.get_available_video_devices())
                            continue

                        # 일반 패키지 렌더링
                        has_running = any(f["status"] == "RUNNING" for f in pkg_data["files"].values())

                        with ui.expansion(pkg, icon="folder", value=has_running).classes(
                            "w-full bg-white shadow-sm border border-slate-200 rounded-xl px-2 py-1"
                        ).props('header-class="text-lg font-bold text-slate-800"'):
                            with ui.grid(columns=1).classes("w-full gap-2 pt-2 pb-1 md:grid-cols-2"):
                                for file_name, info in sorted(pkg_data["files"].items()):

                                    is_running = info["status"] == "RUNNING"

                                    # 버튼 스타일 설정
                                    btn_text = "RUNNING" if is_running else "RUN"
                                    btn_color = "red-6" if is_running else "green-6"
                                    btn_icon = "pause" if is_running else "play_arrow"

                                    with ui.row().classes("w-full items-center justify-between px-4 py-2 bg-slate-50 rounded-lg border border-slate-200"):
                                        ui.label(file_name).classes("text-sm font-medium text-slate-800 truncate flex-1 mr-2")

                                        btn = ui.button(btn_text, color=btn_color, icon=btn_icon).props("dense unelevated").classes("min-w-[110px]")
                                        info["button"] = btn

                                        # 버튼 클릭 이벤트 연결
                                        btn.on("click", lambda _, p=pkg, f=file_name: NiceGUIRos_instance.toggle_launch(p, f))

        except RuntimeError:
            # 페이지를 이동해서 UI 요소가(Client가) 삭제된 경우입니다.
            # 에러를 무시하고, 참조를 초기화하여 다음 호출 때 방어합니다.
            # print("[DEBUG] UI update skipped: Client deleted (Page changed)")
            self.launch_list_container = None
            self.search_status_label = None

    def render_usb_cam_controller(self, pkg, file_name, devices):
        info = self.launch_files_data[pkg]["files"][file_name]
        with ui.card().classes("w-full shadow-lg border border-indigo-200"):
            ui.label(f"**📷 {file_name}**").classes("text-lg font-bold mb-3")
            with ui.row().classes("w-full items-center gap-4"):
                v = ui.select(options=devices, value=devices[0] if devices else "", label="Device").classes("w-1/2")
                f = ui.select(options=["yuyv", "mjpeg"], value="yuyv", label="Format").classes("w-1/2")
            btn = ui.button(
                "RUNNING" if info["status"] == "RUNNING" else "RUN",
                icon="pause" if info["status"] == "RUNNING" else "play_arrow",
                color="red-6" if info["status"] == "RUNNING" else "green-6",
            ).classes("w-full mt-4")
            info["button"] = btn
            btn.on("click", lambda: self.toggle_usb_cam_launch(v.value, f.value))

    # ============================================================
    # Image & Topic (갱신 로직 강화)
    # ============================================================
    def update_topic_list(self):
        if not self.ros_client or not self.ros_client.is_connected:
            return

        def _cb(res):
            opts = []
            topic_dict = {}  # [추가] 토픽 저장을 위한 임시 딕셔너리

            # 이름과 타입을 가져와서 이미지 토픽만 필터링
            for n, t in zip(res.get("topics", []), res.get("types", [])):
                if "sensor_msgs/Image" in str(t) or "sensor_msgs/CompressedImage" in str(t):
                    opts.append(n)
                    topic_dict[n] = t  # [추가] 타입 정보도 저장

            # ★★★ [핵심 수정] 클래스 변수에 저장해야 다른 페이지에서 갖다 쓸 수 있음! ★★★
            self.available_image_topics = topic_dict

            # (기존) 메인 페이지 UI 업데이트
            if self.image_topic_select:
                sorted_opts = sorted(opts)
                self.image_topic_select.set_options(sorted_opts)

                if self.image_topic_select.value and self.image_topic_select.value not in sorted_opts:
                    self.image_topic_select.set_value(None)
                    self.unsubscribe_current_image_topic()
                    ui.notify("선택된 영상 토픽이 사라졌습니다.", type="warning")

        try:
            self.ros_client.get_topics(callback=_cb)
            print("[DEBUG] Requesting topic update...")
        except:
            pass

    @staticmethod
    def _to_bytes(data):
        if isinstance(data, (bytes, bytearray)):
            return bytes(data)
        if isinstance(data, str):
            try:
                return base64.b64decode(data)
            except:
                return data.encode("latin1")
        return str(data).encode("latin1")

    def subscribe_image_topic(self, topic):
        self.unsubscribe_current_image_topic()
        if not topic or not self.ros_client or not self.ros_client.is_connected:
            return
        is_comp = "compressed" in topic.lower()
        self.current_image_topic = roslibpy.Topic(self.ros_client, topic, "sensor_msgs/CompressedImage" if is_comp else "sensor_msgs/Image")

        def cb(msg):
            if not cv2 or not np:
                return
            try:
                data = msg.get("data")
                if not data:
                    return
                jb = None
                if is_comp:
                    jb = self._to_bytes(data)
                else:
                    h, w = msg["height"], msg["width"]
                    raw = self._to_bytes(data)
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    c = 1 if len(arr) == h * w else 3
                    img = arr.reshape((h, w, c)) if c > 1 else arr.reshape((h, w))
                    if c == 3 and msg.get("encoding") == "rgb8":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    ok, enc = cv2.imencode(".jpg", img)
                    if ok:
                        jb = enc.tobytes()
                if jb:
                    self.streamer.update_frame(jb)
                    self.last_frame_time = time.time()
                    self.needs_label_update = True
            except:
                pass

        self.current_image_topic.subscribe(cb)
        self.is_image_subscribed = True
        if self.image_display:
            self.image_display.set_source("/video_feed")

    def unsubscribe_current_image_topic(self):
        if self.current_image_topic:
            try:
                self.current_image_topic.unsubscribe()
            except:
                pass
        self.current_image_topic = None
        self.is_image_subscribed = False
        self.streamer.stop_streaming()
        if self.image_display:
            self.image_display.set_source(self.current_image_src)
        if self.last_frame_label:
            self.last_frame_label.set_text("Last frame: -")

    def update_ui_from_thread(self):
        if self.needs_label_update and self.last_frame_time and self.last_frame_label:
            self.last_frame_label.set_text(f"Last frame: {time.strftime('%H:%M:%S', time.localtime(self.last_frame_time))}")
            self.needs_label_update = False

    # ============================================================
    # LLM Chat Logic
    # ============================================================
    def setup_llm_ros_topics(self):
        if not self.ros_client or not self.ros_client.is_connected:
            return
        if self.llm_sub:
            try:
                self.llm_sub.unsubscribe()
            except:
                pass
            self.llm_sub = None

        self.llm_pub = roslibpy.Topic(self.ros_client, "/gui/input", "std_msgs/String")
        self.llm_sub = roslibpy.Topic(self.ros_client, "/rkllama/output", "std_msgs/String")

        def cb(msg):
            if self.main_loop:
                self.main_loop.call_soon_threadsafe(self._handle_llm_callback, msg)

        self.llm_sub.subscribe(cb)

    def _handle_llm_callback(self, msg):
        token = msg.get("data", "")
        if not token:
            return

        # --- 스트림 시작 ---
        if token == "__LLM_START__":
            self.is_llm_streaming = True
            self.current_llm_stream_text = ""
            self._remove_loading_indicator()

            self.chat_log_buffer.append({"sender": "LLM", "text": ""})
            self._render_chat_log()
            return

        # --- 스트림 종료 ---
        if token == "__LLM_END__":
            self.is_llm_streaming = False
            self.current_llm_stream_text = ""
            self._remove_loading_indicator()
            return

        # --- 스트리밍 중 토큰 ---
        if not self.is_llm_streaming:
            # 방어: START 없이 들어온 토큰 무시
            return

        self.current_llm_stream_text += token
        self.chat_log_buffer[-1]["text"] = self.current_llm_stream_text
        self._render_chat_log()

    def _render_chat_log(self):
        if not self.chat_log_container:
            return
        html = ""
        for item in self.chat_log_buffer:
            s, t = item["sender"], item["text"]
            align = "justify-end" if s == "User" else "justify-start"
            bg = "bg-green-100" if s == "User" else ("bg-blue-100" if s == "LLM" else "bg-gray-100 italic")
            html += f'<div class="flex w-full mb-2 {align}"><div class="px-3 py-2 rounded-lg max-w-[70%] {bg}"><div class="text-xs text-gray-500">[{s}]</div><div>{t}</div></div></div>'
        self.chat_log_container.content = html
        try:
            js = f'document.getElementById("{self.chat_log_container.id}").scrollTop = document.getElementById("{self.chat_log_container.id}").scrollHeight'
            self.chat_log_container.client.run_javascript(js)
        except:
            pass

    def _append_to_chat_log(self, s, t):
        self.chat_log_buffer.append({"sender": s, "text": t})
        self._render_chat_log()

    def _append_loading_indicator(self):
        if not self.is_waiting_for_llm:
            self.is_waiting_for_llm = True
            self.chat_log_buffer.append({"sender": "Loading", "text": "..."})
            self._render_chat_log()

    def _remove_loading_indicator(self):
        if self.is_waiting_for_llm:
            if self.chat_log_buffer and self.chat_log_buffer[-1]["sender"] == "Loading":
                self.chat_log_buffer.pop()
            self.is_waiting_for_llm = False
            self._render_chat_log()

    async def on_send_command(self):
        if not self.chat_input or not self.llm_pub:
            return
        txt = self.chat_input.value.strip()
        if not txt:
            return

        self.is_llm_streaming = False
        self.current_llm_stream_text = ""
        self._append_to_chat_log("User", txt)
        self._append_loading_indicator()
        self.llm_pub.publish(roslibpy.Message({"data": txt}))
        self.chat_input.set_value("")

    def start_local_camera(self, device_path: str, width=1280, height=720):
        self.stop_local_camera()
        self.unsubscribe_current_image_topic()

        try:
            dev_idx = int("".join(filter(str.isdigit, device_path)))

            # [수정 1] 백엔드 지정(CAP_V4L2) 및 포맷 강제 설정 제거
            # 일부 카메라에서 MJPG 강제 시 호환성 문제가 생길 수 있어 기본값으로 엽니다.
            cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)

            # 해상도 설정
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            # [옵션] MJPG 시도해보고 싶다면 아래 주석 해제 (하지만 일단은 주석 처리 추천)
            # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            if cap.isOpened():
                self.local_cam = cap
                self.is_local_cam_running = True
                self.local_cam_task = asyncio.create_task(self._local_cam_loop())
                ui.notify(f"Camera {device_path} Opened (Loop Started).", type="positive")
                print(f"[DEBUG] Camera {device_path} isOpened() == True")
            else:
                ui.notify(f"Failed to open {device_path}", type="negative")
                print(f"[DEBUG] Camera {device_path} isOpened() == False")

        except Exception as e:
            ui.notify(f"Error opening camera: {e}", type="negative")
            print(f"[ERROR] {e}")

    def stop_local_camera(self):
        """로컬 카메라를 닫습니다."""
        self.is_local_cam_running = False
        if self.local_cam:
            self.local_cam.release()
            self.local_cam = None
        self.streamer.stop_streaming()

    async def _local_cam_loop(self):
        print("[DEBUG] Local Cam Loop Started")
        loop = asyncio.get_running_loop()

        fail_count = 0
        while self.is_local_cam_running and self.local_cam and self.local_cam.isOpened():
            try:
                # [수정 2] 카메라 읽기 비동기 처리
                ret, frame = await loop.run_in_executor(None, self.local_cam.read)

                if ret:
                    fail_count = 0  # 성공하면 카운트 리셋
                    ok, encoded = cv2.imencode(".jpg", frame)
                    if ok:
                        self.streamer.update_frame(encoded.tobytes())
                        self.last_frame_time = time.time()
                        self.needs_label_update = True
                else:
                    # 읽기 실패 시 로그 출력 (너무 자주는 말고)
                    fail_count += 1
                    if fail_count % 30 == 0:
                        print(f"[WARN] Camera open but read() failed. (Count: {fail_count})")

                await asyncio.sleep(0.015)

            except Exception as e:
                print(f"[ERROR] Cam loop error: {e}")
                break
        print("[DEBUG] Local Cam Loop Ended")

    def set_camera_prop(self, prop_name, value):
        """카메라 속성값(밝기, 노출 등)을 변경합니다."""
        if not self.local_cam or not self.local_cam.isOpened():
            return

        prop_id = CV_PROPS.get(prop_name)
        if prop_id is not None:
            self.local_cam.set(prop_id, value)
            # 설정 적용을 위해 잠시 대기 (일부 카메라는 버퍼 비우기 필요)
            # threading을 쓰지 않고 간단히 처리
            print(f"[CAM] Set {prop_name} to {value}")

    def get_camera_prop(self, prop_name):
        """현재 카메라 속성값을 읽어옵니다."""
        if not self.local_cam or not self.local_cam.isOpened():
            return 0
        prop_id = CV_PROPS.get(prop_name)
        if prop_id is not None:
            return self.local_cam.get(prop_id)
        return 0


# ============================================================
# Singleton Instance
# ============================================================
NiceGUIRos_instance = NiceGUIRos()
