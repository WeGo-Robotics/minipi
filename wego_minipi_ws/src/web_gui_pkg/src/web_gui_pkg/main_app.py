#!/usr/local/bin/python3.10
# web_gui_pkg/main_app.py

from nicegui import ui, app
from pathlib import Path
import asyncio

# 모듈 임포트
from .core_logic import NiceGUIRos_instance, mjpeg_streamer

# 페이지 모듈 임포트 (이때 @ui.page 데코레이터가 실행되어 라우팅이 등록됨)
from .page_launch import main_page
from .page_llm_chat import llm_chat_page
from .page_cam_setting import cam_setting_page
from .page_startup import startup_page

# 정적 파일 설정
current_dir = Path(__file__).parent
static_folder_path = current_dir / "static"
app.add_static_files("/static", str(static_folder_path))


# 비디오 스트림 엔드포인트
@app.get("/video_feed")
async def video_feed():
    return await mjpeg_streamer.stream_frames()


# 앱 시작 시 ROS 연결 시도 (Async Task로 실행)
@app.on_startup
async def on_app_startup():
    print("INFO: Application starting up...")
    await NiceGUIRos_instance.setup_ros_client()


# 앱 종료 시 정리
@app.on_shutdown
def shutdown_ros_client():
    print("INFO: Shutting down...")
    NiceGUIRos_instance.stop_all_launches()
    NiceGUIRos_instance.unsubscribe_current_image_topic()

    # 로컬 카메라가 켜져 있다면 종료
    if NiceGUIRos_instance.is_local_cam_running:
        NiceGUIRos_instance.stop_local_camera()

    if NiceGUIRos_instance.ros_client and NiceGUIRos_instance.ros_client.is_connected:
        try:
            NiceGUIRos_instance.ros_client.terminate()
            print("INFO: ROS Client terminated.")
        except Exception as e:
            print(f"WARN: Error during ROS termination (safe to ignore): {e}")


def main():
    ui.run(
        host="0.0.0.0",
        port=8089,
        title="ROS Launch Manager",
        reload=False,
        show=False,
    )


if __name__ == "__main__":
    main()
