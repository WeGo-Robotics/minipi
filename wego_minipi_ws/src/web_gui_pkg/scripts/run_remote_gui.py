#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from nicegui import ui, app
import os
import sys

# 1) src/web_gui_pkg 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../src")
sys.path.append(src_path)

# 2) 새 원격 페이지 + core_logic 임포트
from web_gui_pkg.page_remote import RemoteControlPage
from web_gui_pkg.core_logic import NiceGUIRos_instance, mjpeg_streamer


# 3) /remote 페이지 등록
remote_page = RemoteControlPage()
remote_page.create_page()


# 4) 루트('/') 접속 시 자동으로 /remote로 이동
@ui.page("/")
def root_redirect():
    ui.timer(0.1, lambda: ui.navigate.to("/remote"), once=True)


def main():
    # static 폴더 등록
    static_path = os.path.join(current_dir, "../src/web_gui_pkg/static")
    app.add_static_files("/static", static_path)

    # MJPEG 스트리밍 엔드포인트 (/video_feed)
    from fastapi.responses import StreamingResponse

    @app.get("/video_feed")
    async def video_feed():
        if NiceGUIRos_instance.is_image_subscribed or NiceGUIRos_instance.is_local_cam_running:
            return await mjpeg_streamer.stream_frames()
        else:
            return StreamingResponse(iter([]), status_code=204)

    @app.on_shutdown
    def shutdown():
        NiceGUIRos_instance.unsubscribe_current_image_topic()
        if NiceGUIRos_instance.ros_client and NiceGUIRos_instance.ros_client.is_connected:
            NiceGUIRos_instance.ros_client.terminate()
            print("INFO: ROS Client terminated.")

    ui.run(
        host="0.0.0.0",
        port=9099,
        title="Mini Pi Remote",
        reload=False,
        show=False,
    )


if __name__ == "__main__":
    main()
