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
    ROSBRIDGE_HOST = "0.0.0.0"  # rosbridge 호스트 적절히 설정 (로봇 측의 rosbridge와 주소 맞아야 함)
    ROSBRIDGE_PORT = 9090  # rosbridge 호스트 적절히 설정 (로봇 측의 rosbridge와 주소 맞아야 함)
    MAX_LOG_LINES = 100
    HOME_DIR = Path(os.path.expanduser("~"))
    ROS_SRC_DIR = HOME_DIR / "soccer_ws" / "src"
    ROS_SETUP_COMMAND = (
        f"source /opt/ros/noetic/setup.bash; " f"source {HOME_DIR}/realsense_ws/devel/setup.bash; " f"source {HOME_DIR}/soccer_ws/devel/setup.bash"
    )


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
        self.log_container: Optional[ui.label] = None
        self.image_topic_select: Optional[ui.select] = None
        self.image_display: Optional[ui.image] = None
        self.last_frame_label: Optional[ui.label] = None

        # ROS Client
        self.ros_client: Optional[roslibpy.Ros] = None
        self.ros_connected = False
        self._last_ros_connected = False

        # Logs
        self.current_log_process: Optional[subprocess.Popen] = None
        self.log_buffer: List[str] = []
        self.log_executor_task: Optional[asyncio.Task] = None

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

        # Cam Setting
        self.local_cam = None
        self.is_local_cam_running = False
        self.local_cam_task = None

        # Main Loop Capture
        self.main_loop = None
        app.on_startup(self._capture_loop)

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
    def get_workspace_packages(self) -> List[str]:
        src_path = ROS_SRC_DIR
        print(f"DEBUG: Scanning packages in: {src_path}")
        found_packages = set()
        if src_path.exists():
            for item in src_path.iterdir():
                if item.is_dir() and (item / "package.xml").exists():
                    found_packages.add(item.name)
        else:
            print(f"ERROR: Source path {src_path} not found!")
        explicit_packages = ["realsense2_camera", "usb_cam", "using_llm"]  # 사용자 정의 추가 패키지
        found_packages.update(explicit_packages)
        return sorted(list(found_packages))  # src 경로 내 패키지 + 추가 패키지

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
            launch_cmd = f"roslaunch {pkg} {file_name}"
            if args:
                launch_cmd += " " + " ".join(args)
            cmd = f"{ROS_SETUP_COMMAND} && {launch_cmd}"
            p = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            info["process"] = p
            info["status"] = "RUNNING"
            if info.get("button"):
                info["button"].set_text("RUNNING")
                info["button"].props("color=red icon=pause")
            self.current_log_process = p
            self.log_buffer = []
            if not self.log_executor_task or self.log_executor_task.done():
                self.log_executor_task = asyncio.create_task(self.monitor_log())
            ui.notify(f"Started {file_name}", type="positive")

            # ★★★ [핵심 수정] 실행 3초 후 토픽 목록 자동 갱신 ★★★
            # ROS 노드가 뜨고 토픽을 advertise하는데 시간이 걸리므로 지연 실행
            ui.timer(3.0, self.update_topic_list, once=True)

            if pkg == "using_llm" and file_name == "llm_gui.launch":
                ui.navigate.to("/LLM_chat", new_tab=False)

            elif pkg == "ball_tracker_pkg" and file_name == "cam_setting.launch":
                ui.navigate.to("/cam_setting", new_tab=False)

        except Exception as e:
            ui.notify(f"Fail: {e}", type="negative")

    def stop_launch(self, pkg: str, file_name: str, info: Dict[str, Any]):
        p = info.get("process")
        if p:
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=3)
            except:
                pass
        info["status"] = "STOPPED"
        info["process"] = None
        if info.get("button"):
            info["button"].set_text("RUN")
            info["button"].props("color=green icon=play_arrow")

        # ★★★ [핵심 수정] 정지 1초 후 토픽 목록 자동 갱신 ★★★
        # 죽은 토픽을 목록에서 제거하기 위함
        ui.timer(1.0, self.update_topic_list, once=True)

    def stop_all_launches(self):
        for pkg, pkg_data in self.launch_files_data.items():
            for file_name, info in pkg_data["files"].items():
                if info["status"] == "RUNNING":
                    self.stop_launch(pkg, file_name, info)
        self.render_launch_list()

    async def monitor_log(self):
        if not self.current_log_process:
            return
        proc, loop = self.current_log_process, asyncio.get_event_loop()
        while proc and proc.poll() is None:
            try:
                out = await loop.run_in_executor(None, proc.stdout.readline)
                err = await loop.run_in_executor(None, proc.stderr.readline)
                updated = False
                if out:
                    self.log_buffer.append("[OUT] " + out.decode(errors="replace").strip())
                    updated = True
                if err:
                    self.log_buffer.append("[ERR] " + err.decode(errors="replace").strip())
                    updated = True
                if updated and self.log_container:
                    if len(self.log_buffer) > MAX_LOG_LINES:
                        self.log_buffer = self.log_buffer[-MAX_LOG_LINES:]
                    self.log_container.set_text("\n".join(self.log_buffer))
                    try:
                        self.log_container.client.run_javascript(
                            f'document.getElementById("{self.log_container.id}").parentElement.scrollTop = document.getElementById("{self.log_container.id}").parentElement.scrollHeight'
                        )
                    except:
                        pass
                await asyncio.sleep(0.01)
            except:
                break

    # ============================================================
    # Render Logic
    # ============================================================
    # core_logic.py 내부의 메서드

    def render_launch_list(self):
        # 컨테이너가 없으면(초기화 안됨) 무시
        if not self.launch_list_container:
            return

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
            # 이름과 타입을 가져와서 이미지 토픽만 필터링
            for n, t in zip(res.get("topics", []), res.get("types", [])):
                if "sensor_msgs/Image" in str(t) or "sensor_msgs/CompressedImage" in str(t):
                    opts.append(n)

            # UI 업데이트
            if self.image_topic_select:
                sorted_opts = sorted(opts)
                self.image_topic_select.set_options(sorted_opts)

                # 현재 선택된 토픽이 사라졌으면 선택 해제 및 구독 중지
                if self.image_topic_select.value and self.image_topic_select.value not in sorted_opts:
                    self.image_topic_select.set_value(None)
                    self.unsubscribe_current_image_topic()
                    ui.notify("선택된 영상 토픽이 사라졌습니다.", type="warning")

                # (옵션) 선택된 것이 없고, 가능한 토픽이 1개 이상이면 첫 번째 자동 선택?
                # -> 사용자가 직접 선택하는게 나을 수 있어 자동선택은 주석처리
                # elif not self.image_topic_select.value and sorted_opts:
                #     self.image_topic_select.set_value(sorted_opts[0])
                #     self.subscribe_image_topic(sorted_opts[0])

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
        answer = msg.get("data", "ERR")
        if self.chat_log_buffer and self.chat_log_buffer[-1]["text"] == answer:
            self._remove_loading_indicator()
            return
        self._remove_loading_indicator()
        self._append_to_chat_log("LLM", answer)

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
