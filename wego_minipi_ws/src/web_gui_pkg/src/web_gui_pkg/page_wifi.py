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
        self.ssid_label = None  # 와이파이 이름 표시용 라벨

        # 기존 파일 경로
        self.script_path = os.path.join(HOME_DIR, "soccer_ws", "ble_wifi_setup.py")

    def get_current_ssid(self):
        """현재 연결된 와이파이 SSID를 가져옵니다."""
        try:
            # nmcli 명령어로 활성 연결 확인
            # -t: terse output (스크립트용)
            # -f ACTIVE,SSID: 활성여부와 SSID만 출력
            result = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # 출력 예시:
            # no:Wifi_A
            # yes:My_Home_Wifi
            # no:Wifi_B
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    if line.startswith("yes:"):
                        return line.split(":", 1)[1]
            return "연결 안 됨"
        except Exception:
            return "확인 불가"

    def start_ble_process(self):
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
            self.process = subprocess.Popen(
                ["sudo", sys.executable, self.script_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, preexec_fn=os.setsid
            )
            self.is_running = True
            ui.notify("BLE 모드 시작 (주변에서 검색 가능)", type="positive")
            asyncio.create_task(self.read_output())

        except Exception as e:
            ui.notify(f"실행 실패: {e}", type="negative")

    def stop_ble_process(self):
        if self.process and self.process.poll() is None:
            try:
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
            except Exception:
                break
            await asyncio.sleep(0.01)

        self.is_running = False
        if self.process and self.process.poll() is not None:
            if self.log_element:
                self.log_element.set_text(self.log_content + "\n[Process Terminated]")


wifi_manager = WifiPage()


@ui.page("/wifi")
def wifi_setup_page():

    # 주기적으로 SSID 업데이트하는 함수
    def update_ssid_label():
        ssid = wifi_manager.get_current_ssid()
        if wifi_manager.ssid_label:
            wifi_manager.ssid_label.set_text(ssid)
            # 연결 상태에 따라 색상 변경
            if ssid in ["연결 안 됨", "확인 불가"]:
                wifi_manager.ssid_label.classes("text-red-500")
            else:
                wifi_manager.ssid_label.classes("text-green-500")

    # ----- 헤더 -----
    with ui.header().classes("bg-slate-900 shadow-lg"):
        with ui.row().classes("w-full items-center h-full max-w-screen-xl mx-auto px-4"):
            ui.html("""<img src="/static/wego_logo.png" style="height: 32px;">""")
            ui.label(ROBOT_NAME + " Wi-Fi Setup").classes("text-white font-bold text-lg")
            ui.space()
            ui.button("메인으로", icon="home", color="indigo-6").props("flat").on("click", lambda: ui.navigate.to("/", new_tab=False))

    # ----- 메인 내용 -----
    with ui.column().classes("p-4 w-full max-w-screen-md mx-auto space-y-4"):

        # [추가된 부분] 현재 상태 표시 카드
        with ui.card().classes("w-full flex flex-row items-center justify-between px-6 py-4 border-l-4 border-blue-500"):
            with ui.column().classes("gap-0"):
                ui.label("현재 연결된 Wi-Fi").classes("text-lg font-bold text-gray-700")
                ui.label("실시간 상태입니다.").classes("text-xs text-gray-400")

            # SSID 표시 라벨
            wifi_manager.ssid_label = ui.label("불러오는 중...").classes("text-2xl font-bold")

        # BLE 제어 카드
        with ui.card().classes("w-full"):
            ui.label("📡 블루투스 와이파이 설정").classes("text-xl font-bold mb-2")
            ui.label("이 기능을 켜면 스마트폰 웹 블루투스를 통해 로봇의 와이파이를 설정할 수 있습니다.").classes("text-gray-600 mb-4")

            with ui.row().classes("w-full gap-4"):
                ui.button("BLE 모드 시작", icon="bluetooth", color="green", on_click=wifi_manager.start_ble_process).classes("flex-1 h-12 text-lg")
                ui.button("중지", icon="stop", color="red", on_click=wifi_manager.stop_ble_process).classes("flex-1 h-12 text-lg")

        # 로그 창
        # with ui.card().classes("w-full bg-black text-green-400 p-4 font-mono h-64 overflow-y-auto"):
        #     wifi_manager.log_element = ui.label("대기 중...").classes("whitespace-pre-wrap")

    # 3초마다 와이파이 상태 갱신
    ui.timer(3.0, update_ssid_label)
