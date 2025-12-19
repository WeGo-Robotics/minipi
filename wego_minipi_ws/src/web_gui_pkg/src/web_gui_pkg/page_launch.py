# web_gui_pkg/page_launch.py

from nicegui import ui
from pathlib import Path
from .core_logic import NiceGUIRos_instance
from .config import ROBOT_NAME


# ============================================================
# 외부(core_logic)에서 호출 가능한 UI 진입점
# ============================================================


def render_launch_list_ui():
    NiceGUIRos_instance.render_launch_list()


# ============================================================
# Workspace Add Dialog
# ============================================================


def workspace_add_dialog():
    with ui.dialog() as dialog, ui.card().classes("w-[420px]"):
        ui.label("워크스페이스 추가").classes("text-lg font-bold")

        path_input = ui.input(
            label="ROS 워크스페이스 경로",
            placeholder="~/wego_minipi_ws",
        ).classes("w-full")

        with ui.row().classes("justify-end w-full mt-4"):
            ui.button("취소", on_click=dialog.close).props("flat")

            def on_add():
                if not path_input.value:
                    ui.notify("경로를 입력하세요", type="warning")
                    return

                try:
                    NiceGUIRos_instance.add_workspace(Path(path_input.value))
                    ui.notify("워크스페이스 추가 완료", type="positive")
                    dialog.close()
                except Exception as e:
                    ui.notify(f"추가 실패: {e}", type="negative")

            ui.button("추가", color="primary", on_click=on_add)

    dialog.open()


# ============================================================
# Workspace Remove Confirm
# ============================================================


def confirm_remove(ws_path):
    with ui.dialog() as d, ui.card():
        ui.label("정말 삭제하시겠습니까?").classes("font-bold")
        ui.label(str(ws_path)).classes("font-mono text-sm text-gray-600")

        with ui.row().classes("justify-end mt-4"):
            ui.button("취소", on_click=d.close)
            ui.button(
                "삭제",
                color="red",
                on_click=lambda: (
                    NiceGUIRos_instance.remove_workspace(ws_path),
                    ui.notify("워크스페이스 삭제 완료", type="warning"),
                    d.close(),
                ),
            )
    d.open()


# ============================================================
# Workspace Manage Dialog
# ============================================================


def workspace_manage_dialog():
    def load_state():
        store = NiceGUIRos_instance._load_store()
        return (
            set(store.get("pinned_packages", [])),
            set(store.get("hidden_packages", [])),
        )

    pinned, hidden = load_state()

    with ui.dialog() as dialog, ui.card().classes("w-[720px] p-4"):
        ui.label("워크스페이스 관리").classes("text-lg font-bold mb-4")

        for ws in NiceGUIRos_instance.load_workspaces():
            ws_path = ws["path"]
            role = ws["role"]

            # -------- Package Expansion --------
            with ui.expansion(value=False).classes("w-full bg-white border rounded-xl shadow-sm mb-3") as exp:

                # ===== Header (여기에 전부 넣는다) =====
                with exp.add_slot("header"):
                    with ui.row().classes("w-full items-center gap-3 px-2"):
                        ui.icon("folder").classes("text-blue-600")

                        ui.label(str(ws_path)).classes("font-mono text-sm flex-1 truncate")

                        role_select = (
                            ui.select(
                                options=["SYSTEM", "EXTERNAL", "USER"],
                                value=role.upper(),
                            )
                            .props("outlined dense")
                            .classes("w-[120px]")
                        )

                        if role == "system":
                            role_select.disable()

                        role_select.on(
                            "update:model-value",
                            lambda e, p=ws_path: NiceGUIRos_instance.set_workspace_role(p, e.value.lower()),
                        )

                        if role == "user":
                            ui.button(
                                icon="delete",
                                color="red",
                                on_click=lambda p=ws_path: confirm_remove(p),
                            ).props("flat dense")

                # ===== Body =====
                src = Path(ws_path) / "src"
                if not src.exists():
                    ui.label("src 디렉토리 없음").classes("text-xs text-gray-400 px-4 py-2")
                    continue

                pkgs = sorted(p.name for p in src.iterdir() if (p / "package.xml").exists())

                with ui.column().classes("w-full px-4 pb-3"):
                    for pkg in pkgs:
                        with ui.row().classes("w-full items-center py-1 hover:bg-slate-50 rounded"):
                            ui.label(pkg).classes("text-sm flex-1")

                            if role == "user":
                                ui.checkbox(
                                    "Hide",
                                    value=pkg in hidden,
                                    on_change=lambda e, p=pkg: NiceGUIRos_instance.set_package_hidden(p, e.value),
                                )
                            else:
                                ui.checkbox(
                                    "Pin",
                                    value=pkg in pinned,
                                    on_change=lambda e, p=pkg: NiceGUIRos_instance.set_package_pinned(p, e.value),
                                )

        with ui.row().classes("justify-end mt-4"):
            ui.button("닫기", on_click=dialog.close)

    dialog.open()


# ============================================================
# Main Page
# ============================================================


@ui.page("/")
def main_page():
    # ----- Header -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html('<img src="/static/wego_logo.png" style="height:32px;margin-right:8px;">')
            ui.label(f"{ROBOT_NAME} ROS Launch Manager").classes("text-white font-bold text-lg")
            ui.space()

            ui.button("와이파이 재조정", icon="wifi", color="blue-6").props("flat").on("click", lambda: ui.navigate.to("/wifi"))

            ui.button("부팅 설정", icon="save_as", color="green-6").props("flat").on("click", lambda: ui.navigate.to("/startup"))

            ui.button(
                "모두 정지",
                icon="stop",
                color="red-6",
            ).props("flat").classes(
                "font-semibold"
            ).on("click", lambda _: NiceGUIRos_instance.stop_all_launches())

    # ----- Tabs -----
    with ui.tabs().classes("w-full mt-4") as tabs:
        ui.tab("LAUNCHES")
        ui.tab("LOGS")
        ui.tab("VIDEO")

    with ui.tab_panels(tabs, value="LAUNCHES").classes("w-full p-4"):
        # ================= LAUNCHES =================
        with ui.tab_panel("LAUNCHES"):
            with ui.column().classes("w-full items-center"):
                with ui.row().classes("w-full max-w-5xl justify-end gap-2 mb-2"):
                    ui.button("워크스페이스 관리", icon="settings", on_click=workspace_manage_dialog)
                    ui.button("워크스페이스 추가", icon="add", on_click=workspace_add_dialog)

                with ui.column().classes("w-full max-w-5xl space-y-4") as launch_col:
                    NiceGUIRos_instance.launch_list_container = launch_col
                    NiceGUIRos_instance.search_status_label = ui.label("Initializing...").classes("text-sm text-gray-500 mt-4")

        # ================= LOGS =================
        with ui.tab_panel("LOGS"):
            ui.label("실행 중인 launch별 로그").classes("text-sm text-gray-600 mb-2")
            with ui.column().classes("w-full space-y-3") as logs_col:
                NiceGUIRos_instance.logs_container = logs_col

        # ================= VIDEO =================
        with ui.tab_panel("VIDEO"):
            with ui.column().classes("w-full items-center"):
                with ui.row().classes("w-full max-w-lg justify-center items-center"):
                    NiceGUIRos_instance.image_topic_select = ui.select(
                        options=[],
                        label="영상 토픽 선택",
                        on_change=lambda e: NiceGUIRos_instance.subscribe_image_topic(e.value),
                    ).classes("w-1/2")

                    ui.button(
                        "새로고침",
                        icon="refresh",
                        on_click=NiceGUIRos_instance.update_topic_list,
                    ).classes("w-1/4")

                NiceGUIRos_instance.image_display = ui.image(NiceGUIRos_instance.current_image_src).classes(
                    "w-full max-w-screen-lg border-2 border-dashed min-h-[300px]"
                )

                NiceGUIRos_instance.last_frame_label = ui.label("Last frame: (none)").classes("mt-2 text-sm text-gray-700")

    # ----- Footer -----
    with ui.footer().classes("justify-center items-center h-6 bg-gray-200"):
        ui.label("Mini Pi Launch Manager v1.0").classes("text-xs font-semibold text-gray-700 mr-4")
        ui.label("|").classes("text-xs text-gray-400")
        ui.label("© 2025 wego robotics").classes("text-xs text-gray-700 ml-4")

    # 초기화
    ui.timer(0.5, NiceGUIRos_instance.initialize, once=True)
