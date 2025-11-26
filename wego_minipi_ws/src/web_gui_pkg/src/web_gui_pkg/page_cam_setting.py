# web_gui_pkg/page_cam_setting.py

from nicegui import ui
import json
import os
import time
import glob
from .core_logic import NiceGUIRos_instance
from .config import ROBOT_NAME, HOME_DIR

# 기본 범위값
RANGES = {
    "BRIGHTNESS": (-64, 64), "CONTRAST": (0, 64), "SATURATION": (0, 128),
    "HUE": (-40, 40), "GAIN": (0, 100), "EXPOSURE": (1, 5000), 
    "WB_TEMPERATURE": (2800, 6500),
}
DEFAULTS = {
    "BRIGHTNESS": 0, "CONTRAST": 32, "SATURATION": 64, "HUE": 0, "GAIN": 0, 
    "EXPOSURE": 157, "WB_TEMPERATURE": 4600,
}

@ui.page("/cam_setting")
def cam_setting_page():
    
    # 프리셋 저장 폴더 설정 (없으면 생성)
    PRESET_DIR = os.path.join(HOME_DIR, "soccer_ws", "cam_preset")
    os.makedirs(PRESET_DIR, exist_ok=True)
    
    controls = {}
    control_container = None

    # ----- 헤더 -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html(
                """<img src="/static/wego_logo.png" style="height: 32px; width: auto; max-width: 150px; margin-right: 8px;">""",
            )
            ui.label(ROBOT_NAME + " Camera Setting Viewer").classes("text-white font-bold text-lg")
            ui.space()
            ui.button("메인으로", icon="home", color="indigo-6").props("flat").classes("font-semibold").on(
                "click", lambda: ui.navigate.to("/", new_tab=False)
            )
            ui.button("모두 정지", icon="stop", color="red-6").props("flat").classes("font-semibold").on(
                "click", lambda _: NiceGUIRos_instance.stop_all_launches()
            )
    
    # ============================================================
    # 내부 로직 함수들
    # ============================================================
    def get_all_sources():
        local_devs = NiceGUIRos_instance.get_available_video_devices()
        topics = list(NiceGUIRos_instance.available_image_topics.keys())
        return sorted(local_devs) + sorted(topics)

    def refresh_sources():
        NiceGUIRos_instance.update_topic_list()
        new_opts = get_all_sources()
        device_select.set_options(new_opts)
        ui.notify("소스 목록을 갱신했습니다.")

    def on_open_camera():
        dev = device_select.value
        if not dev:
            ui.notify("디바이스(또는 토픽)를 선택해주세요.", type="warning")
            return
        
        # --- 케이스 1: 로컬 장치 ---
        if dev.startswith("/dev/"):
            NiceGUIRos_instance.unsubscribe_current_image_topic()
            NiceGUIRos_instance.start_local_camera(dev)
            
            if NiceGUIRos_instance.is_local_cam_running:
                enable_controls(True)
                load_current_values()
                ui.query('img[src^="/video_feed"]').props(f'src="/video_feed?t={time.time()}"')
            else:
                ui.notify(f"{dev}를 열 수 없습니다.", type="negative")

        # --- 케이스 2: ROS 토픽 ---
        else:
            NiceGUIRos_instance.stop_local_camera()
            NiceGUIRos_instance.subscribe_image_topic(dev)
            enable_controls(False)
            ui.query('img[src^="/video_feed"]').props(f'src="/video_feed?t={time.time()}"')
            ui.notify(f"ROS 토픽 구독: {dev}", type="positive")

    def enable_controls(enable: bool):
        if not control_container: return
        control_container.visible = enable
        if not enable:
            status_label.set_text("ROS 토픽 모드에서는 하드웨어 제어가 불가능합니다.")
            status_label.classes("text-red-500")
        else:
            status_label.set_text("하드웨어 제어 가능")
            status_label.classes("text-green-500")

    def load_current_values():
        if not NiceGUIRos_instance.is_local_cam_running: return
        for key, slider in controls.items():
            if key in ["AUTO_EXPOSURE", "AUTO_WB"]: continue
            val = NiceGUIRos_instance.get_camera_prop(key)
            if val != -1: slider.set_value(val)

    def on_prop_change(name, value):
        if NiceGUIRos_instance.is_local_cam_running:
            NiceGUIRos_instance.set_camera_prop(name, value)

    def on_auto_change(name, value):
        if not NiceGUIRos_instance.is_local_cam_running: return
        if name == "AUTO_EXPOSURE":
            cam_val = 0.75 if value else 0.25
            NiceGUIRos_instance.set_camera_prop("AUTO_EXPOSURE", cam_val)
            controls["EXPOSURE"].disable() if value else controls["EXPOSURE"].enable()
            controls["GAIN"].disable() if value else controls["GAIN"].enable()
        elif name == "AUTO_WB":
            cam_val = 1.0 if value else 0.0
            NiceGUIRos_instance.set_camera_prop("AUTO_WB", cam_val)
            controls["WB_TEMPERATURE"].disable() if value else controls["WB_TEMPERATURE"].enable()

    # ============================================================
    # 프리셋 저장/로드 로직 (다이얼로그 사용)
    # ============================================================
    def open_save_dialog():
        """저장 다이얼로그 열기"""
        with ui.dialog() as dialog, ui.card():
            ui.label("프리셋 저장").classes("text-lg font-bold")
            name_input = ui.input("파일 이름", placeholder="예: night_mode").classes("w-full")
            
            def _save():
                name = name_input.value.strip()
                if not name:
                    ui.notify("이름을 입력해주세요.", type="warning")
                    return
                if not name.endswith(".json"): name += ".json"
                
                path = os.path.join(PRESET_DIR, name)
                
                # 데이터 수집
                data = {k: v.value for k, v in controls.items()}
                data["DEVICE"] = device_select.value
                
                try:
                    with open(path, "w") as f: json.dump(data, f, indent=2)
                    ui.notify(f"저장 완료: {name}", type="positive")
                    dialog.close()
                except Exception as e:
                    ui.notify(f"저장 실패: {e}", type="negative")

            with ui.row().classes("w-full justify-end"):
                ui.button("취소", on_click=dialog.close).props("flat")
                ui.button("저장", on_click=_save)
        dialog.open()

    def open_load_dialog():
        """로드 다이얼로그 열기"""
        # 저장된 파일 목록 가져오기
        files = glob.glob(os.path.join(PRESET_DIR, "*.json"))
        file_names = [os.path.basename(f) for f in files]
        
        if not file_names:
            ui.notify("저장된 프리셋 파일이 없습니다.", type="warning")
            return

        with ui.dialog() as dialog, ui.card().classes("w-80"):
            ui.label("프리셋 불러오기").classes("text-lg font-bold")
            
            selected_file = ui.select(file_names, label="파일 선택", value=file_names[0]).classes("w-full")
            
            def _load():
                fname = selected_file.value
                if not fname: return
                
                path = os.path.join(PRESET_DIR, fname)
                try:
                    with open(path, "r") as f: data = json.load(f)
                    
                    # 장치 선택 적용
                    if "DEVICE" in data:
                        dev = data["DEVICE"]
                        if dev not in device_select.options:
                            current_opts = device_select.options
                            device_select.set_options(current_opts + [dev])
                        device_select.set_value(dev)
                    
                    # 값 적용
                    for k, v in data.items():
                        if k in controls:
                            controls[k].set_value(v)
                            
                    ui.notify(f"로드 완료: {fname} (Open Camera를 눌러 적용하세요)", type="positive")
                    dialog.close()
                except Exception as e:
                    ui.notify(f"로드 실패: {e}", type="negative")

            with ui.row().classes("w-full justify-end"):
                ui.button("취소", on_click=dialog.close).props("flat")
                ui.button("불러오기", on_click=_load)
        dialog.open()

    # ============================================================
    # UI 레이아웃
    # ============================================================
    with ui.row().classes("w-full p-4 gap-4"):
        # [왼쪽] 비디오 프리뷰 및 연결
        with ui.column().classes("w-full lg:w-7/12"):
            with ui.card().classes("w-full"):
                ui.label("Camera Connection").classes("font-bold")
                with ui.row().classes("w-full items-center gap-2"):
                    initial_opts = get_all_sources()
                    device_select = ui.select(initial_opts, label="Source", with_input=True).classes("w-60")
                    ui.button(icon="refresh", on_click=refresh_sources).props("flat dense").tooltip("소스 목록 새로고침")
                    
                    ui.button("Open Camera", icon="videocam", on_click=on_open_camera).classes("ml-auto")
                    ui.button("Close", icon="close", color="red", on_click=NiceGUIRos_instance.stop_local_camera)
            
            ui.image("/video_feed").classes("w-full border-2 border-slate-300 rounded mt-4 min-h-[400px] bg-black")

        # [오른쪽] 컨트롤 패널
        with ui.column().classes("w-full lg:w-4/12 space-y-4"):
            with ui.card().classes("w-full"):
                ui.label("Settings").classes("font-bold mb-2")
                status_label = ui.label("장치를 열어주세요").classes("text-sm text-gray-500 mb-2")

                with ui.column().classes("w-full") as control_container:
                    # Auto Switches
                    with ui.row().classes("gap-4 mb-4"):
                        s_exp = ui.switch("Auto Exposure", on_change=lambda e: on_auto_change("AUTO_EXPOSURE", e.value))
                        s_wb = ui.switch("Auto WB", on_change=lambda e: on_auto_change("AUTO_WB", e.value))
                        controls["AUTO_EXPOSURE"] = s_exp
                        controls["AUTO_WB"] = s_wb

                    # Sliders
                    def create_slider(name):
                        min_v, max_v = RANGES.get(name, (0, 100))
                        def_v = DEFAULTS.get(name, 0)
                        ui.label(name).classes("text-xs font-bold text-gray-500 mt-2")
                        sl = ui.slider(min=min_v, max=max_v, value=def_v, step=1, 
                                    on_change=lambda e: on_prop_change(name, e.value)).props("label-always")
                        controls[name] = sl

                    create_slider("BRIGHTNESS")
                    create_slider("CONTRAST")
                    create_slider("SATURATION")
                    create_slider("HUE")
                    create_slider("GAIN")
                    create_slider("EXPOSURE")
                    create_slider("WB_TEMPERATURE")

            # 저장/로드 버튼 (수정됨)
            with ui.row().classes("w-full justify-between"):
                ui.button("Save Preset", icon="save", on_click=open_save_dialog).classes("w-1/2 mr-1")
                ui.button("Load Preset", icon="file_open", color="secondary", on_click=open_load_dialog).classes("w-1/2 ml-1")