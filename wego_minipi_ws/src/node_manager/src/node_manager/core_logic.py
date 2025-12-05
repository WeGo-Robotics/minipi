#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ============================
# Launch Manager 전용 core_logic.py
# ============================

import os
import subprocess
import signal
import threading
import asyncio
import base64
import time
from typing import Dict, Any, Optional, List, Set
from pathlib import Path
import glob

# NiceGUI
from nicegui import ui, app

# ====================================
# 설정 로딩 (config.py가 없으면 기본값 사용)
# ====================================
try:
    from .config import (
        ROBOT_NAME,
        ROSBRIDGE_HOST,
        ROSBRIDGE_PORT,
        MAX_LOG_LINES,
        ROS_SETUP_COMMAND,
        ROS_SRC_DIR
    )
except ImportError:
    print("[WARN] config.py 로드 실패 → 기본값 사용")
    HOME = Path.home()
    ROBOT_NAME = "Mini Pi"
    ROSBRIDGE_HOST = "0.0.0.0"
    ROSBRIDGE_PORT = 9090
    MAX_LOG_LINES = 200
    ROS_SRC_DIR = HOME / "soccer_ws" / "src"
    ROS_SETUP_COMMAND = (
        f"source /opt/ros/noetic/setup.bash; "
        f"source {HOME_DIR}/realsense_ws/devel/setup.bash; "
        f"source {HOME}/soccer_ws/devel/setup.bash"
    )

# ====================================
# 라이브러리 로드
# ====================================
try:
    import roslibpy
    import cv2
    import numpy as np
except ImportError as e:
    print(f"[FATAL] Missing dependency: {e}")
    roslibpy = None
    cv2 = None
    np = None


# ====================================
# MJPEG 스트리머
# ====================================
class MjpegStreamer:
    def __init__(self):
        self._latest_jpeg: Optional[bytes] = None
        self._lock = threading.Lock()
        self.is_streaming = False

    def update_frame(self, jpeg: bytes):
        with self._lock:
            self._latest_jpeg = jpeg

    def get_frame(self):
        with self._lock:
            return self._latest_jpeg

    async def stream_frames(self):
        """FastAPI StreamingResponse로 반환"""
        self.is_streaming = True
        mjpeg_header = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

        async def gen():
            while self.is_streaming:
                frame = self.get_frame()
                if frame:
                    yield mjpeg_header + frame + b"\r\n"
                await asyncio.sleep(1 / 30)

        from fastapi.responses import StreamingResponse
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    def stop(self):
        self.is_streaming = False


mjpeg_streamer = MjpegStreamer()


# ====================================
# 핵심 관리 클래스
# ====================================
class NiceGUIRos:
    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.launch_files_data: Dict[str, Any] = {}
        self._launch_loaded = False
        self._ui_initialized = False

        # UI 참조
        self.launch_list_container = None
        self.search_status_label = None
        self.log_container = None

        # ROS Client
        self.ros_client: Optional[roslibpy.Ros] = None
        self.ros_connected = False
        self._last_conn_state = False

        # 로그
        self.current_log_process: Optional[subprocess.Popen] = None
        self.log_buffer: List[str] = []
        self.log_task: Optional[asyncio.Task] = None

        # 영상/이미지
        self.image_topic_select = None
        self.image_display = None
        self.last_frame_label = None
        self.current_image_topic = None
        self.available_image_topics = {}
        self.is_image_subscribed = False
        self.current_image_src = "data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs="

        # main loop 캡처
        app.on_startup(self._capture_loop)

    async def _capture_loop(self):
        try:
            self.main_loop = asyncio.get_running_loop()
        except RuntimeError:
            pass

    # ============================
    # ROS 연결
    # ============================
    def on_connection(self, *args):
        print("[ROS] CONNECTED")
        self.ros_connected = True
        self.update_topic_list()

    def on_close(self, *args):
        print("[ROS] DISCONNECTED")
        self.ros_connected = False

    def setup_ros_client(self):
        """rosbridge 연결"""
        if not roslibpy or self.ros_client:
            return

        try:
            self.ros_client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
            self.ros_client.on_ready(self.on_connection)

            # =========================
            # ✔ FIX: roslibpy close handler cross-version
            # =========================
            if hasattr(self.ros_client, "on_close"):
                self.ros_client.on_close(self.on_close)
            else:
                try:
                    self.ros_client.on('close', self.on_close)
                except Exception:
                    print("[WARN] Could not bind ROS close handler")

            threading.Thread(
                target=self.ros_client.run_forever,
                daemon=True
            ).start()

        except Exception as e:
            print("[ROS INIT ERROR]", e)


    # ============================
    # 초기화
    # ============================
    def initialize(self):
        if not self._ui_initialized:
            self._ui_initialized = True
            self.setup_ros_client()
            ui.timer(0.5, self.poll_ros_status)
            ui.timer(5.0, self.update_topic_list)

        if self._launch_loaded:
            self.render_launch_list()
        else:
            asyncio.create_task(self.start_launch_search())

    def poll_ros_status(self):
        """상태 변화 시 notify"""
        connected = bool(self.ros_client and self.ros_client.is_connected)
        if connected and not self._last_conn_state:
            ui.notify("ROS Connected", type="positive")
        if (not connected) and self._last_conn_state:
            ui.notify("ROS Disconnected", type="warning")
        self._last_conn_state = connected

    # ============================
    # Launch 파일 탐색
    # ============================
    def get_workspace_packages(self) -> List[str]:
        src_path = ROS_SRC_DIR
        print(f"DEBUG: Scanning packages in: {src_path}")
        found_packages = set()
        if src_path.exists():
            for item in src_path.iterdir():
                if item.is_dir() and (item / "package.xml").exists():
                    found_packages.add(item.name)

        # ★ 추가적인 강제 패키지 목록
        explicit_packages = ["realsense2_camera", "usb_cam", "using_llm", "sim2real_master"]
        found_packages.update(explicit_packages)

        return sorted(list(found_packages))

    def find_launch_files(self, packages):
        result = {}
        for pkg in packages:
            try:
                cmd = f"{ROS_SETUP_COMMAND} && rospack find {pkg}"
                o = subprocess.run(cmd, shell=True, executable="/bin/bash",
                                   capture_output=True, text=True)
                path = o.stdout.strip()
                if not path:
                    continue

                launch_dir = os.path.join(path, "launch")
                if not os.path.isdir(launch_dir):
                    continue

                launch_files = [
                    f for f in os.listdir(launch_dir)
                    if f.endswith(".launch") or f.endswith(".xml")
                ]

                if launch_files:
                    result[pkg] = {
                        "path": path,
                        "files": {
                            f: {"status": "STOPPED", "process": None, "button": None}
                            for f in launch_files
                        },
                    }

            except Exception as e:
                print(f"[WARN] search error in {pkg}: {e}")
        return result

    async def start_launch_search(self):
        if self.search_status_label:
            self.search_status_label.set_text("Searching workspace...")
        loop = asyncio.get_event_loop()
        pkgs = await loop.run_in_executor(None, self.get_workspace_packages)
        self.launch_files_data = await loop.run_in_executor(None, self.find_launch_files, pkgs)
        self._launch_loaded = True
        self.render_launch_list()
        if self.search_status_label:
            self.search_status_label.set_text("Search complete")

    # ============================
    # 실행 중 launch 확인
    # ============================
    def get_running_launches(self):
        try:
            out = subprocess.check_output("ps aux | grep roslaunch | grep -v grep", shell=True).decode()
            running = set()
            for line in out.splitlines():
                for token in line.split():
                    if token.endswith(".launch"):
                        running.add(token)
            return running
        except:
            return set()

    # ============================
    # Launch 실행/정지
    # ============================
    def run_launch(self, pkg, file, info):
        try:
            cmd = f"{ROS_SETUP_COMMAND} && roslaunch {pkg} {file}"
            p = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 preexec_fn=os.setsid)
            info["process"] = p
            info["status"] = "RUNNING"
            if info.get("button"):
                info["button"].set_text("RUNNING")
                info["button"].props("color=red icon=pause")

            self.current_log_process = p
            self.log_buffer = []
            if not self.log_task or self.log_task.done():
                self.log_task = asyncio.create_task(self.monitor_log())

            ui.notify(f"{file} started", type="positive")
        except Exception as e:
            ui.notify(f"Launch failed: {e}", type="negative")

    def stop_launch(self, pkg, file, info):
        p = info.get("process")
        if p:
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=3)
            except:
                pass

        info["process"] = None
        info["status"] = "STOPPED"
        if info.get("button"):
            info["button"].set_text("RUN")
            info["button"].props("color=green icon=play_arrow")

        ui.notify(f"{file} stopped", type="warning")

    def toggle_launch(self, pkg, file):
        info = self.launch_files_data[pkg]["files"][file]
        if info["status"] == "STOPPED":
            self.run_launch(pkg, file, info)
        else:
            self.stop_launch(pkg, file, info)

    def stop_all_launches(self):
        for pkg, pkgdata in self.launch_files_data.items():
            for file, info in pkgdata["files"].items():
                if info["status"] == "RUNNING":
                    self.stop_launch(pkg, file, info)
        self.render_launch_list()

    # ============================
    # 로그 모니터링
    # ============================
    async def monitor_log(self):
        proc = self.current_log_process
        if not proc:
            return
        loop = asyncio.get_event_loop()

        while proc.poll() is None:
            out = await loop.run_in_executor(None, proc.stdout.readline)
            err = await loop.run_in_executor(None, proc.stderr.readline)

            updated = False
            if out:
                self.log_buffer.append(out.decode(errors="replace").rstrip())
                updated = True
            if err:
                self.log_buffer.append(err.decode(errors="replace").rstrip())
                updated = True

            if updated and self.log_container:
                if len(self.log_buffer) > MAX_LOG_LINES:
                    self.log_buffer = self.log_buffer[-MAX_LOG_LINES:]
                self.log_container.set_text("\n".join(self.log_buffer))

            await asyncio.sleep(0.03)

    # ============================
    # UI 렌더링
    # ============================
    def render_launch_list(self):
        if not self.launch_list_container:
            return

        running_set = self.get_running_launches()

        # running 상태 업데이트
        for pkg, pkg_data in self.launch_files_data.items():
            for file_name, info in pkg_data["files"].items():
                info["status"] = "RUNNING" if file_name in running_set else "STOPPED"

        try:
            self.launch_list_container.clear()

            with self.launch_list_container:
                if self.search_status_label:
                    self.search_status_label.visible = False

                # launch 파일 없을 때
                if not self.launch_files_data:
                    ui.label("No launch files found.").classes("text-center mt-4")
                    return

                # 메인 컬럼
                with ui.column().classes("w-full max-w-5xl mx-auto space-y-3"):
                    for pkg, pkg_data in sorted(self.launch_files_data.items()):

                        # -------------------------
                        # USB_CAM 특수 UI 제거
                        # -------------------------
                        # if pkg == "usb_cam" and "usb_cam-test.launch" in pkg_data["files"]:
                        #     self.render_usb_cam_controller(...) 
                        #     continue

                        has_running = any(f["status"] == "RUNNING" for f in pkg_data["files"].values())

                        # 패키지 expansion
                        with ui.expansion(
                            pkg, 
                            icon="folder",
                            value=has_running
                        ).classes(
                            "w-full bg-white shadow-sm border border-slate-200 rounded-xl px-2 py-1"
                        ).props(
                            'header-class="text-lg font-bold text-slate-800"'
                        ):

                            # 2-column grid
                            with ui.grid(columns=1).classes(
                                "w-full gap-2 pt-2 pb-1 md:grid-cols-2"
                            ):
                                for file_name, info in sorted(pkg_data["files"].items()):

                                    is_running = info["status"] == "RUNNING"

                                    btn_text = "RUNNING" if is_running else "RUN"
                                    btn_color = "red-6" if is_running else "green-6"
                                    btn_icon = "pause" if is_running else "play_arrow"

                                    # 파일 행(row)
                                    with ui.row().classes(
                                        "w-full items-center justify-between px-4 py-2 "
                                        "bg-slate-50 rounded-lg border border-slate-200"
                                    ):
                                        ui.label(file_name).classes(
                                            "text-sm font-medium text-slate-800 truncate flex-1 mr-2"
                                        )

                                        btn = ui.button(
                                            btn_text,
                                            color=btn_color,
                                            icon=btn_icon
                                        ).props(
                                            "dense unelevated"
                                        ).classes(
                                            "min-w-[110px]"
                                        )

                                        info["button"] = btn
                                        btn.on(
                                            "click",
                                            lambda _, p=pkg, f=file_name: 
                                                NiceGUIRos_instance.toggle_launch(p, f)
                                        )

        except RuntimeError:
            self.launch_list_container = None
            self.search_status_label = None


    # ============================
    # 이미지 토픽 처리
    # ============================
    def _to_bytes(self, data):
        if isinstance(data, (bytes, bytearray)):
            return bytes(data)
        if isinstance(data, str):
            try:
                return base64.b64decode(data)
            except:
                return data.encode()
        return bytes(str(data), "utf-8")

    def update_topic_list(self):
        if not self.ros_client or not self.ros_client.is_connected:
            return

        def cb(res):
            topics = []
            topic_dict = {}
            for n, t in zip(res["topics"], res["types"]):
                if "Image" in t:
                    topics.append(n)
                    topic_dict[n] = t
            self.available_image_topics = topic_dict

            if self.image_topic_select:
                self.image_topic_select.set_options(sorted(topics))

        try:
            self.ros_client.get_topics(callback=cb)
        except:
            pass

    def subscribe_image_topic(self, topic):
        self.unsubscribe_current_image_topic()
        if not topic or not self.ros_client or not self.ros_client.is_connected:
            return

        is_compressed = "compressed" in topic.lower()
        msg_type = "sensor_msgs/CompressedImage" if is_compressed else "sensor_msgs/Image"

        self.current_image_topic = roslibpy.Topic(self.ros_client, topic, msg_type)

        def cb(msg):
            if not cv2:
                return

            data = msg.get("data")
            if not data:
                return

            jb = None
            if is_compressed:
                jb = self._to_bytes(data)
            else:
                try:
                    h, w = msg["height"], msg["width"]
                    raw = self._to_bytes(data)
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    img = arr.reshape((h, w, 3))
                    ok, enc = cv2.imencode(".jpg", img)
                    if ok:
                        jb = enc.tobytes()
                except:
                    return

            if jb:
                mjpeg_streamer.update_frame(jb)
                if self.last_frame_label:
                    self.last_frame_label.set_text(time.strftime("%H:%M:%S"))

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
        mjpeg_streamer.stop()


# ============================================================
# Singleton Instance
# ============================================================
NiceGUIRos_instance = NiceGUIRos()
