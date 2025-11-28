#!/usr/bin/env python3

from nicegui import ui, app
from pathlib import Path
import sys
import os

# 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../src")
sys.path.append(src_path)

from web_gui_pkg.core_logic import NiceGUIRos_instance, mjpeg_streamer
import web_gui_pkg.page_launch
import web_gui_pkg.page_llm_chat
import web_gui_pkg.page_cam_setting

# ========================================================
# 1. [추가] 방금 만든 컨트롤 페이지 파일을 불러옵니다.
import web_gui_pkg.page_control 
# ========================================================

def main():
    # 정적 파일 서빙 설정
    static_path = os.path.join(current_dir, "../src/web_gui_pkg/static")
    app.add_static_files("/static", static_path)

    # MJPEG 스트리밍 엔드포인트
    from fastapi.responses import StreamingResponse
    
    @app.get("/video_feed")
    async def video_feed():
        if NiceGUIRos_instance.is_image_subscribed or NiceGUIRos_instance.is_local_cam_running:
            return await mjpeg_streamer.stream_frames()
        else:
            return StreamingResponse(iter([]), status_code=204)

    # 종료 핸들러
    @app.on_shutdown
    def shutdown():
        NiceGUIRos_instance.stop_all_launches()
        NiceGUIRos_instance.unsubscribe_current_image_topic()
        if NiceGUIRos_instance.ros_client and NiceGUIRos_instance.ros_client.is_connected:
            NiceGUIRos_instance.ros_client.terminate()
            print("INFO: ROS Client terminated.")

    # ========================================================
    # 2. [추가] 컨트롤 페이지를 실제로 생성하고 등록합니다.
    # 이 코드가 실행되어야 http://.../control 주소가 생깁니다.
    control_page = web_gui_pkg.page_control.ControlPage()
    control_page.create_page()
    # ========================================================

    # NiceGUI 실행
    ui.run(
        host="0.0.0.0",
        port=8089,
        title="ROS Launch Manager",
        reload=False,
        show=False,
    )

if __name__ == "__main__":
    main()