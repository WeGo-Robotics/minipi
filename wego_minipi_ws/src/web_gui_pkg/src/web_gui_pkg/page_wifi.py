# src/web_gui_pkg/src/web_gui_pkg/page_wifi.py

from nicegui import ui
import os
import subprocess
import signal
import asyncio
import sys
from .config import ROBOT_NAME, HOME_DIR


class WifiPage:
    def __init__(self):
        self.process = None
        self.log_content = ""
        self.log_element = None
        self.is_running = False

        # [수정됨] 기존에 존재하는 파일 경로를 직접 지정
        # 위치: ~/soccer_ws/ble_wifi_setup.py
        self.script_path = os.path.join(HOME_DIR, "soccer_ws", "ble_wifi_setup.py")

    def start_ble_process(self):
        # 파일이 실제로 있는지 확인
        if not os.path.exists(self.script_path):
            ui.notify(f"파일을 찾을 수 없습니다: {self.script_path}", type="negative")
            return

        if self.is_running:
            ui.notify("이미 실행 중입니다.", type="warning")
            return

        self.log_content = ""
        if self.log_element:
            self.log_element.set_text("")

        try:
            # sudo 권한으로 실행 (btmgmt, nmcli 등 필요)
            self.process = subprocess.Popen(
                ["sudo", sys.executable, self.script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,  # 프로세스 그룹 생성 (kill 용이)
            )
            self.is_running = True
            ui.notify("BLE 모드 시작 (주변에서 검색 가능)", type="positive")

            # 로그 읽기 시작
            asyncio.create_task(self.read_output())

        except Exception as e:
            ui.notify(f"실행 실패: {e}", type="negative")

    def stop_ble_process(self):
        if self.process and self.process.poll() is None:
            try:
                # sudo로 실행된 프로세스 그룹 전체 종료
                os.system(f"sudo kill -TERM -{self.process.pid}")
                self.process.wait(timeout=2)
            except:
                pass

        self.is_running = False
        self.process = None
        ui.notify("BLE 모드 종료", type="info")

    async def read_output(self):
        while self.is_running and self.process:
            try:
                line = await asyncio.to_thread(self.process.stdout.readline)
                if not line:
                    break
                self.log_content += line
                if self.log_element:
                    self.log_element.set_text(self.log_content)
                    # 최신 로그가 보이도록 자동 스크롤 기능 추가 가능
            except Exception:
                break
            await asyncio.sleep(0.01)

        self.is_running = False
        if self.process and self.process.poll() is not None:
            # 의도치 않게 종료된 경우 알림 (stop 버튼 누른 경우 제외)
            if self.log_element:
                self.log_element.set_text(self.log_content + "\n[Process Terminated]")


wifi_manager = WifiPage()


@ui.page("/wifi")
def wifi_setup_page():
    # ----- 헤더 -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html("""<img src="/static/wego_logo.png" style="height: 32px;">""")
            ui.label(ROBOT_NAME + " Wi-Fi Setup").classes("text-white font-bold text-lg")
            ui.space()
            ui.button("메인으로", icon="home", color="indigo-6").props("flat").on("click", lambda: ui.navigate.to("/", new_tab=False))

    # ----- 메인 내용 -----
    with ui.column().classes("p-4 w-full max-w-screen-md mx-auto"):

        # 파일 경로 확인용 (디버깅)
        # ui.label(f"Target Script: {wifi_manager.script_path}").classes("text-xs text-gray-400 mb-2")

        with ui.card().classes("w-full mb-4"):
            ui.label("📡 블루투스 와이파이 설정").classes("text-xl font-bold mb-2")
            ui.label("이 기능을 켜면 스마트폰 웹 블루투스를 통해 로봇의 와이파이를 설정할 수 있습니다.").classes("text-gray-600 mb-4")

            with ui.row().classes("w-full gap-4"):
                ui.button("BLE 모드 시작", icon="bluetooth", color="green", on_click=wifi_manager.start_ble_process).classes("flex-1 h-12 text-lg")
                ui.button("중지", icon="stop", color="red", on_click=wifi_manager.stop_ble_process).classes("flex-1 h-12 text-lg")

        # 로그 창
        with ui.card().classes("w-full bg-black text-green-400 p-4 font-mono h-64 overflow-y-auto"):
            wifi_manager.log_element = ui.label("대기 중...").classes("whitespace-pre-wrap")
