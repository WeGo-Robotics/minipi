# web_gui_pkg/page_llm_chat.py

from nicegui import ui
from .core_logic import NiceGUIRos_instance
from .config import ROBOT_NAME

@ui.page("/LLM_chat")
def llm_chat_page():
    # ROS 연결 확인 및 LLM 토픽 설정
    if NiceGUIRos_instance.ros_connected:
        NiceGUIRos_instance.setup_llm_ros_topics()
    else:
        ui.notify("ROS Bridge가 연결되지 않았습니다. LLM 통신이 불가능합니다.", type="warning")

    # ----- 헤더 -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html(
                """<img src="/static/wego_logo.png" style="height: 32px; width: auto; max-width: 150px; margin-right: 8px;">""",
                
            )
            ui.label(ROBOT_NAME + " LLM Chat Viewer").classes("text-white font-bold text-lg")
            ui.space()
            ui.button("메인으로", icon="home", color="indigo-6").props("flat").classes("font-semibold").on(
                "click", lambda: ui.navigate.to("/", new_tab=False)
            )
            ui.button("모두 정지", icon="stop", color="red-6").props("flat").classes("font-semibold").on(
                "click", lambda _: NiceGUIRos_instance.stop_all_launches()
            )

    # ----- 메인 컨텐츠 -----
    with ui.column().classes("p-4 w-full max-w-screen-xl mx-auto"):
        with ui.row().classes("w-full gap-4 flex-wrap lg:flex-nowrap lg:justify-between"):
            
            # 1. 왼쪽 (영상)
            with ui.column().classes("w-full lg:w-7/12"):
                with ui.row().classes("w-full items-center gap-2"):
                    NiceGUIRos_instance.image_topic_select = ui.select(
                        options=list(NiceGUIRos_instance.available_image_topics.keys()),
                        label="영상 토픽 선택",
                        on_change=lambda e: NiceGUIRos_instance.subscribe_image_topic(e.value),
                    ).classes("flex-grow")
                    ui.button("새로고침", icon="refresh", on_click=NiceGUIRos_instance.update_topic_list)

                NiceGUIRos_instance.image_display = ui.image(NiceGUIRos_instance.current_image_src).classes(
                    "w-full h-[400px] border-2 border-dashed border-slate-300 rounded-lg min-h-[240px]"
                )
                NiceGUIRos_instance.last_frame_label = ui.label("Last frame: (none)").classes("mt-2 text-sm text-gray-700")

            # 2. 오른쪽 (대화)
            with ui.column().classes("w-full lg:w-5/12"):
                NiceGUIRos_instance.chat_log_container = ui.html(
                    # 버퍼 내용을 HTML로 변환하는 로직은 core_logic 내부 메서드로 처리하거나 여기서 직접 렌더링
                    "" 
                ).classes("w-full h-[500px] bg-white border border-slate-400 rounded-lg overflow-y-auto p-2")
                NiceGUIRos_instance.chat_log_container.style("font-family: 'Noto Sans KR', sans-serif; font-size: 14pt;")
                
                # 초기 로드 시 채팅 기록 복구 (core_logic에 _render_chat_log 메서드가 있다고 가정)
                NiceGUIRos_instance._render_chat_log()

                with ui.card().classes("w-full shadow-lg p-3"):
                    ui.label("User Input (LLM Command)").classes("text-sm font-semibold mb-2")
                    with ui.row().classes("w-full items-center gap-2"):
                        NiceGUIRos_instance.chat_input = ui.input(
                            placeholder="메시지 입력 후 Send",
                        ).classes("flex-grow").on('keydown.enter', NiceGUIRos_instance.on_send_command)
                        
                        NiceGUIRos_instance.send_button = ui.button("Send", icon="send").on(
                            'click', NiceGUIRos_instance.on_send_command
                        )

    # ----- 푸터 -----
    with ui.footer().classes("justify-center items-center h-6 bg-gray-200"):
        ui.label("Mini Pi Launch Manager v1.0").classes("text-xs font-semibold text-gray-700 mr-4")