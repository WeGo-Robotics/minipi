#!/usr/bin/env python3

from nicegui import ui, app
from pathlib import Path

# 패키지 모듈 임포트
# 주의: src 폴더가 PYTHONPATH에 있거나 패키지로 설치되어 있어야 합니다.
# 로컬 테스트 시 sys.path.append 사용 가능
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../src")
sys.path.append(src_path)

from node_manager.core_logic import NiceGUIRos_instance, mjpeg_streamer
import node_manager.page_launch   # 메인 페이지 등록 (@ui.page 데코레이터 실행)


def main():
    # 정적 파일 서빙 설정
    # static 폴더는 scripts/../static 에 있다고 가정
    static_path = os.path.join(current_dir, "../src/node_manager/static")
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

    # NiceGUI 실행
    ui.run(
        host="0.0.0.0",
        port=8089,
        title="ROS Launch Manager",
        reload=False, # ROS 스레드 충돌 방지
        show=False,
    )

if __name__ == "__main__":
    main()