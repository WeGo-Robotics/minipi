#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ============================================================
# 🔧 Twisted / roslibpy signal 충돌 방지
# ============================================================
import os
os.environ.setdefault('TWISTED_DISABLE_SIGNAL_HANDLERS', '1')
os.environ.setdefault('ROSLIBPY_NO_SIGNAL_HANDLERS', '1')
os.environ.setdefault('ROSLIBPY_NO_ROS_CORE', '1')

print("Env:", {
    'TWISTED_DISABLE_SIGNAL_HANDLERS': os.environ.get('TWISTED_DISABLE_SIGNAL_HANDLERS'),
    'ROSLIBPY_NO_SIGNAL_HANDLERS': os.environ.get('ROSLIBPY_NO_SIGNAL_HANDLERS'),
    'ROSLIBPY_NO_ROS_CORE': os.environ.get('ROSLIBPY_NO_ROS_CORE'),
})

import subprocess
import signal
import threading
from typing import Dict, Any, Optional, List
import base64
import time

from nicegui import ui, app
from fastapi.responses import StreamingResponse

# ============================================================
# roslibpy / CV2 임포트
# ============================================================
try:
    import roslibpy
    import asyncio
    import cv2
    import numpy as np
except ImportError:
    print("FATAL: Required libraries (roslibpy, cv2, numpy) not installed.")
    roslibpy = None
    asyncio = None
    cv2 = None
    np = None
    
# ============================================================
# ROS Bridge 설정
# ============================================================
ROSBRIDGE_HOST = "192.168.0.48"
ROSBRIDGE_PORT = 9090

HOME_DIR = os.path.expanduser('~')
ROS_SETUP_COMMAND = (
    f"source /opt/ros/noetic/setup.bash && "
    f"source {HOME_DIR}/soccer_ws/devel/setup.bash"
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_WORKSPACE_PATH = os.path.dirname(SCRIPT_DIR)

MAX_LOG_LINES = 100


# ============================================================
# MJPEG Streamer
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
        
        async def frame_generator():
            mjpeg_header = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
            while self.is_streaming:
                frame = self.get_frame()
                if frame:
                    yield mjpeg_header + frame + b"\r\n"
                await asyncio.sleep(0.033) 

        return StreamingResponse(
            frame_generator(), 
            media_type="multipart/x-mixed-replace; boundary=frame"
        )

    def stop_streaming(self):
        self.is_streaming = False

mjpeg_streamer = MjpegStreamer()


# ============================================================
# MAIN CLASS
# ============================================================
class NiceGUIRos:

    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.launch_files_data: Dict[str, Any] = {}
        self.launch_list_container: Optional[ui.column] = None

        self.ros_client: Optional[roslibpy.Ros] = None
        self.ros_connected = False
        self._last_ros_connected = False

        self.current_log_process: Optional[subprocess.Popen] = None
        self.log_container: Optional[ui.label] = None
        self.log_buffer: List[str] = []
        self.log_executor_task: Optional[asyncio.Task] = None

        self.image_topic_select: Optional[ui.select] = None
        self.image_display: Optional[ui.image] = None
        self.current_image_topic: Optional[roslibpy.Topic] = None
        self.available_image_topics: Dict[str, str] = {}
        self.current_image_src = ("data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs=")
        
        self.streamer: MjpegStreamer = mjpeg_streamer
        self.last_frame_time: Optional[float] = None
        self.last_frame_label: Optional[ui.label] = None
        self.is_image_subscribed = False
        self.needs_label_update = False # UI 업데이트 플래그

        self.status_bar_lbl: Optional[ui.label] = None
        self._initialized = False

    def on_connection(self, *args):
        print("[ROS] CONNECTED (on_ready callback)")
        self.ros_connected = True

    def on_close(self, *args):
        print("[ROS] BRIDGE CLOSED")
        self.ros_connected = False

    def initialize(self):
        if self._initialized: return
        self._initialized = True
        print("INIT START")
        pkgs = self.get_workspace_packages()
        self.launch_files_data = self.find_launch_files(pkgs)
        self.render_launch_list()
        self.setup_ros_client()
        ui.timer(0.5, self.poll_ros_status)
        ui.timer(1.0, self.update_topic_list)

    def setup_ros_client(self):
        if not roslibpy or self.ros_client: return
        print("Creating ROS Client...")
        try:
            self.ros_client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
        except Exception as e:
            print("ROS client init error:", e)
            return
        self.ros_client.on_ready(self.on_connection)
        try: self.ros_client.on_close(self.on_close)
        except Exception: pass 
        def ros_thread():
            print("[ROS] run_forever thread starting")
            try: self.ros_client.run_forever()
            except Exception as e: print("[ROS] run_forever ERROR:", e)
        t = threading.Thread(target=ros_thread, daemon=True)
        t.start()
        print("[ROS] run_forever started in background thread")

    def poll_ros_status(self):
        if not self.status_bar_lbl: return
        connected = bool(self.ros_client and self.ros_client.is_connected)
        if connected and not self._last_ros_connected: ui.notify("ROS Bridge 연결됨 🟢")
        if (not connected) and self._last_ros_connected: ui.notify("ROS Bridge 끊김 🔴", type='warning')
        self._last_ros_connected = connected
        if connected:
            self.status_bar_lbl.set_text(f"ROS Bridge Connected 🟢 ({ROSBRIDGE_HOST}:{ROSBRIDGE_PORT})")
            self.status_bar_lbl.classes('bg-green-500 text-white', remove='bg-red-500 bg-gray-700')
        else:
            self.status_bar_lbl.set_text(f"ROS Bridge Disconnected 🔴 ({ROSBRIDGE_HOST}:{ROSBRIDGE_PORT})")
            self.status_bar_lbl.classes('bg-red-500 text-white', remove='bg-green-500 bg-gray-700')
        self.ros_connected = connected

    def update_ui_from_thread(self):
        if self.needs_label_update and self.last_frame_time and self.last_frame_label:
            t_str = time.strftime('%H:%M:%S', time.localtime(self.last_frame_time))
            self.last_frame_label.set_text(f"Last frame: {t_str}")
            self.needs_label_update = False

    def get_workspace_packages(self) -> List[str]:
        src_path = os.path.join(ROS_WORKSPACE_PATH, 'src')
        result: List[str] = []
        if not os.path.isdir(src_path): return result
        try:
            for item in os.listdir(src_path):
                pkg_path = os.path.join(src_path, item)
                if os.path.isdir(pkg_path) and os.path.exists(os.path.join(pkg_path, 'package.xml')):
                    result.append(item)
        except Exception as e:
            print("get_workspace_packages ERROR:", e)
        return result

    def find_launch_files(self, packages: List[str]) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for pkg in packages:
            files = [] 
            cmd = f"{ROS_SETUP_COMMAND} && rospack find {pkg}"
            try:
                o = subprocess.run(
                    cmd, shell=True, executable='/bin/bash',
                    capture_output=True, text=True, timeout=3
                )
                path = o.stdout.strip()
                if not path or not os.path.isdir(path): continue
                launch_path = os.path.join(path, 'launch')
                if not os.path.isdir(launch_path): continue
                files = [f for f in os.listdir(launch_path) if f.endswith(('.launch', '.xml', '.py'))]
                if files:
                    result[pkg] = {
                        'path': path,
                        'files': {
                            f: {'status': 'STOPPED', 'process': None, 'button': None,}
                            for f in files
                        },
                    }
            except Exception as e:
                print(f"find_launch_files error for {pkg}:", e)
        return result

    def toggle_launch(self, pkg: str, file_name: str):
        info = self.launch_files_data[pkg]['files'][file_name]
        if info['status'] == 'STOPPED': self.run_launch(pkg, file_name, info)
        else: self.stop_launch(pkg, file_name, info)

    def run_launch(self, pkg: str, file_name: str, info: Dict[str, Any]):
        try:
            cmd = f"{ROS_SETUP_COMMAND} && roslaunch {pkg} {file_name}"
            p = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            info['process'] = p
            info['status'] = 'RUNNING'
            if info.get('button'):
                info['button'].set_text('RUNNING')
                info['button'].props('color=red icon=pause')
            self.current_log_process = p
            self.log_buffer = []
            if asyncio:
                if not self.log_executor_task or self.log_executor_task.done():
                    self.log_executor_task = asyncio.create_task(self.monitor_log())
            ui.notify(f"{file_name} 실행됨", type='positive')
        except Exception as e:
            ui.notify(f"실행 실패: {e}", type='negative')

    def stop_launch(self, pkg: str, file_name: str, info: Dict[str, Any]):
        p = info.get('process')
        if p:
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=3)
            except Exception: pass
        info['status'] = 'STOPPED'
        info['process'] = None
        if info.get('button'):
            info['button'].set_text('RUN')
            info['button'].props('color=green icon=play_arrow')
        ui.notify(f"{file_name} 정지됨", type='info')

    def stop_all_launches(self):
        for pkg, pkg_data in self.launch_files_data.items():
            for file_name, info in pkg_data['files'].items():
                self.stop_launch(pkg, file_name, info)
        ui.notify("모든 Launch 정지 완료", type='positive')

    async def monitor_log(self):
        if not self.current_log_process: return
        proc = self.current_log_process
        loop = asyncio.get_event_loop()
        while proc and proc.poll() is None:
            try:
                stdout_line = await loop.run_in_executor(None, proc.stdout.readline)
                stderr_line = await loop.run_in_executor(None, proc.stderr.readline)
                updated = False
                if stdout_line:
                    self.log_buffer.append("[OUT] " + stdout_line.decode().rstrip())
                    updated = True
                if stderr_line:
                    self.log_buffer.append("[ERR] " + stderr_line.decode().rstrip())
                    updated = True
                if updated and self.log_container:
                    if len(self.log_buffer) > MAX_LOG_LINES:
                        self.log_buffer = self.log_buffer[-MAX_LOG_LINES:]
                    text = "\n".join(self.log_buffer)
                    self.log_container.set_text(text)
                await asyncio.sleep(0.01)
            except Exception as e:
                print("monitor_log ERROR:", e)
                break
    
    def render_launch_list(self):
        if not self.launch_list_container: return
        self.launch_list_container.clear()
        if not self.launch_files_data:
            with self.launch_list_container: ui.label("런치 파일 없음")
            return
        with self.launch_list_container:
            for pkg, pkg_data in self.launch_files_data.items():
                ui.label(pkg).classes("text-xl font-bold mt-4 mb-2")
                with ui.column().classes("border p-2 rounded-lg shadow-sm"):
                    for file_name, info in pkg_data['files'].items():
                        with ui.row().classes("w-full justify-between items-center p-1"):
                            ui.label(file_name).classes("text-base")
                            btn = ui.button("RUN", color="green", icon="play_arrow")
                            info['button'] = btn
                            btn.on('click', lambda _, p=pkg, f=file_name: NiceGUIRos_instance.toggle_launch(p, f))

    def update_topic_list(self):
        if not self.ros_client or not self.ros_client.is_connected: return
        def _callback(result):
            names = result.get('topics', [])
            types = result.get('types', [])
            self.available_image_topics = {}
            options: List[str] = []
            for n, t in zip(names, types):
                if 'image' in n.lower() or 'image' in str(t).lower():
                    self.available_image_topics[n] = t
                    options.append(n)
            if self.image_topic_select: self.image_topic_select.set_options(options)
        try:
            self.ros_client.get_topics(callback=_callback)
        except Exception as e:
            print("Topic update error:", e)

    @staticmethod
    def _to_bytes(data):
        if isinstance(data, (bytes, bytearray)): return bytes(data)
        if isinstance(data, list): return bytes(data)
        if hasattr(data, 'tolist'): return bytes(data.tolist())
        if isinstance(data, str):
            try: return base64.b64decode(data)
            except Exception: return data.encode('latin1')
        return str(data).encode('latin1')

    def subscribe_image_topic(self, topic_name: str):
        self.unsubscribe_current_image_topic()
        if not topic_name or not self.ros_client or not self.ros_client.is_connected: return
        is_compressed = topic_name.endswith("/compressed")
        ros_type = ("sensor_msgs/CompressedImage" if is_compressed else "sensor_msgs/Image")
        topic = roslibpy.Topic(self.ros_client, topic_name, ros_type)
        self.current_image_topic = topic

        def cb(msg):
            if not cv2 or not np: return
            try:
                data = msg.get("data")
                if data is None: return
                jpeg_bytes: Optional[bytes] = None
                
                if is_compressed:
                    jpeg_bytes = self._to_bytes(data)
                else:
                    h = msg["height"]; w = msg["width"]; enc = msg.get("encoding", "bgr8")
                    raw = self._to_bytes(data); arr = np.frombuffer(raw, dtype=np.uint8)
                    c = 3 
                    if enc in ("mono8", "8UC1") or len(arr) == h * w: c = 1

                    try: img = arr.reshape((h, w, c)) if c > 1 else arr.reshape((h, w))
                    except ValueError as e:
                        print(f"[IMAGE] reshape error: {e}. Data size: {len(arr)}"); return

                    if c == 3 and enc == "rgb8": img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    elif c == 1: img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

                    ok, enc_img = cv2.imencode(".jpg", img)
                    if not ok: print("[IMAGE] cv2.imencode failed"); return
                    jpeg_bytes = enc_img.tobytes()

                if jpeg_bytes:
                    self.streamer.update_frame(jpeg_bytes)
                    self.last_frame_time = time.time()
                    if self.last_frame_label: self.needs_label_update = True 

            except Exception as e:
                print("Image callback error:", e)

        topic.subscribe(cb)
        self.is_image_subscribed = True
        self.image_display.set_source("/video_feed") 
        ui.notify(f"Subscribed: {topic_name}", type='info')

    def unsubscribe_current_image_topic(self):
        if self.current_image_topic:
            try: self.current_image_topic.unsubscribe()
            except Exception: pass
        self.current_image_topic = None
        self.is_image_subscribed = False
        self.streamer.stop_streaming()
        self.last_frame_time = None
        if self.image_display: self.image_display.set_source(self.current_image_src)
        if self.last_frame_label: self.last_frame_label.set_text("Last frame: (none)")


# ============================================================
# Page UI 구성
# ============================================================

NiceGUIRos_instance = NiceGUIRos()


@ui.page('/')
def main_page():
    
    with ui.header().classes('items-center shadow-md').style("background:#2c3e50"):
        ui.icon('hub', size='md').classes('text-white')
        ui.label('ROS LAUNCH MANAGER').classes('ml-4 text-white')
        ui.space()
        ui.button("모두 정지", color="red",
                  on_click=NiceGUIRos_instance.stop_all_launches)

    with ui.tabs().classes('w-full mt-4') as tabs:
        ui.tab('LAUNCHES'); ui.tab('LOGS'); ui.tab('VIDEO')

    with ui.tab_panels(tabs, value='LAUNCHES').classes('w-full p-4'):

        with ui.tab_panel('LAUNCHES'):
            NiceGUIRos_instance.launch_list_container = ui.column().classes('w-full')
            ui.label("Searching for launch files...")

        with ui.tab_panel('LOGS'):
            ui.label("실행 중인 launch의 로그가 표시됩니다").classes("text-sm text-gray-600")
            box = ui.card().classes('w-full h-96 bg-black text-white p-2 overflow-auto')
            with box:
                NiceGUIRos_instance.log_container = ui.label("로그 대기 중...").classes("whitespace-pre-wrap text-sm")

        with ui.tab_panel('VIDEO'):
            with ui.column().classes('w-full items-center'):
                NiceGUIRos_instance.image_topic_select = ui.select(
                    options=[], label="영상 토픽 선택",
                    on_change=lambda e: NiceGUIRos_instance.subscribe_image_topic(e.value)
                ).classes('w-1/3')

                ui.button("새로고침", icon="refresh", on_click=NiceGUIRos_instance.update_topic_list)

                NiceGUIRos_instance.image_display = ui.image(
                    NiceGUIRos_instance.current_image_src
                ).classes('w-full max-w-screen-lg border-2 border-dashed min-h-[300px]')

                NiceGUIRos_instance.last_frame_label = ui.label("Last frame: (none)").classes("mt-2 text-sm text-gray-700")

    with ui.footer().classes('justify-start items-center h-6 bg-gray-200'):
        NiceGUIRos_instance.status_bar_lbl = ui.label(
            "ROS Bridge 상태 확인 중..."
        ).classes("text-xs font-bold text-gray-700")

    ui.timer(0.5, NiceGUIRos_instance.initialize, once=True)
    ui.timer(0.5, NiceGUIRos_instance.update_ui_from_thread) # UI 업데이트 타이머 등록


# ============================================================
# NiceGUI 실행
# ============================================================
if __name__ == '__main__':
    
    @app.get("/video_feed")
    async def video_feed():
        global mjpeg_streamer
        if NiceGUIRos_instance.is_image_subscribed:
            return await mjpeg_streamer.stream_frames()
        else:
            return StreamingResponse(iter([]), status_code=204)

    @app.on_shutdown
    def shutdown_ros_client():
        global NiceGUIRos_instance
        NiceGUIRos_instance.unsubscribe_current_image_topic()
        if NiceGUIRos_instance.ros_client and NiceGUIRos_instance.ros_client.is_connected:
            NiceGUIRos_instance.ros_client.terminate()
            print("INFO: ROS Client terminated on shutdown.")
        
    ui.run(
        host='0.0.0.0',
        port=8088,
        title='ROS Launch Manager',
        reload=False,
        show=False,
    )