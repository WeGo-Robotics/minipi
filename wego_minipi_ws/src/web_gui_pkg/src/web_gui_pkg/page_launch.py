# web_gui_pkg/page_launch.py

from nicegui import ui
from .core_logic import NiceGUIRos_instance
from .config import ROBOT_NAME


def render_launch_list_ui():
    """외부(core_logic)에서 호출 가능한 UI 렌더링 함수"""
    NiceGUIRos_instance.render_launch_list()


# core_logic에 있던 render_launch_list 메서드를 여기로 옮기거나,
# NiceGUIRos 클래스 안의 메서드를 호출하는 래퍼를 만듭니다.
# 유지보수를 위해 NiceGUIRos 클래스 내의 render_launch_list 로직을
# 아래 render_launch_list_impl 함수로 분리하는 것이 좋습니다.
# 하지만 지금은 원본 코드와의 호환성을 위해 NiceGUIRos_instance.render_launch_list()를 호출한다고 가정합니다.
# (참고: core_logic.py에 render_launch_list 메서드가 포함되어 있어야 합니다.)


@ui.page("/")
def main_page():
    # ----- 헤더 -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html(
                """<img src="/static/wego_logo.png" style="height: 32px; width: auto; max-width: 150px; margin-right: 8px;">""",
            )
            ui.label(ROBOT_NAME + " ROS Launch Manager").classes("text-white font-bold text-lg")
            ui.space()

            ui.button("와이파이 재조정", icon="wifi", color="blue-6").props("flat").on("click", lambda: ui.navigate.to("/wifi", new_tab=False))

            ui.button("부팅 설정", icon="save_as", color="green-6").props("flat").on("click", lambda: ui.navigate.to("/startup", new_tab=False))

            ui.button("모두 정지", icon="stop", color="red-6").props("flat").classes("font-semibold").on(
                "click", lambda _: NiceGUIRos_instance.stop_all_launches()
            )

    # ----- 탭 -----
    with ui.tabs().classes("w-full mt-4") as tabs:
        ui.tab("LAUNCHES")
        ui.tab("LOGS")
        ui.tab("VIDEO")

    with ui.tab_panels(tabs, value="LAUNCHES").classes("w-full p-4"):
        # LAUNCHES 탭
        with ui.tab_panel("LAUNCHES"):
            with ui.column().classes("w-full items-center"):
                with ui.column().classes("w-full max-w-5xl space-y-4") as launch_col:
                    NiceGUIRos_instance.launch_list_container = launch_col
                    NiceGUIRos_instance.search_status_label = ui.label("Initializing...").classes("text-sm text-gray-500 mt-4")
                    NiceGUIRos_instance.search_status_label.visible = True

        # LOGS 탭
        with ui.tab_panel("LOGS"):
            ui.label("실행 중인 launch의 로그가 표시됩니다").classes("text-sm text-gray-600")
            box = ui.card().classes("w-full h-96 bg-black text-white p-2 overflow-auto")
            with box:
                NiceGUIRos_instance.log_container = ui.label("로그 대기 중...").classes("whitespace-pre-wrap text-sm")

        # VIDEO 탭
        with ui.tab_panel("VIDEO"):
            with ui.column().classes("w-full items-center"):
                with ui.row().classes("w-full max-w-lg justify-center items-center"):
                    NiceGUIRos_instance.image_topic_select = ui.select(
                        options=[],
                        label="영상 토픽 선택",
                        on_change=lambda e: NiceGUIRos_instance.subscribe_image_topic(e.value),
                    ).classes("w-1/2")
                    ui.button("새로고침", icon="refresh", on_click=NiceGUIRos_instance.update_topic_list).classes("w-1/4")

                NiceGUIRos_instance.image_display = ui.image(NiceGUIRos_instance.current_image_src).classes(
                    "w-full max-w-screen-lg border-2 border-dashed min-h-[300px]"
                )
                NiceGUIRos_instance.last_frame_label = ui.label("Last frame: (none)").classes("mt-2 text-sm text-gray-700")

    # ----- 푸터 -----
    with ui.footer().classes("justify-center items-center h-6 bg-gray-200"):
        ui.label("Mini Pi Launch Manager v1.0").classes("text-xs font-semibold text-gray-700 mr-4")
        ui.label("|").classes("text-xs text-gray-400")
        ui.label("© 2025 [wego robotics]").classes("text-xs text-gray-700 ml-4")

    # 초기화 호출
    ui.timer(0.5, NiceGUIRos_instance.initialize, once=True)
