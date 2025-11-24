#!/usr.bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import signal
import time
from typing import Dict, Any, Optional, List

from nicegui import ui, app

# --- 1. ROS 환경 설정 ---
ROSBRIDGE_HOST = "127.0.0.1" 
ROSBRIDGE_PORT = 9090

# ROS 1의 워크스페이스 소싱 명령 (실제 환경에서 사용자가 설정한 값)
ROS_SETUP_COMMAND = os.environ.get('ROS_SETUP_COMMAND', "source /opt/ros/noetic/setup.bash && source ~/soccer_ws/devel/setup.bash")

# 현재 스크립트가 ~/soccer_ws/src/niceGUI_ROS.py에 있다고 가정하고 워크스페이스 루트를 찾습니다.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) # .../soccer_ws/src
ROS_WORKSPACE_PATH = os.path.dirname(SCRIPT_DIR)      # .../soccer_ws

# 시스템 기본 패키지 목록
DEFAULT_PACKAGES = ['rviz_visual_tools', 'turtlesim', 'usb_cam'] 

class NiceGUIRos:
    """ROS Bridge 연결 관리, 런치 파일 실행/정지, 상태 관리를 담당하는 핵심 클래스."""
    
    # 인스턴스 변수 초기화만 담당
    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.status_bar_lbl: Optional[ui.label] = None
        self.launch_list_container: Optional[ui.column] = None
        self.launch_files_data: Dict[str, Any] = {}
        self.ros_connected = False
        self.ros_client = None

    # --- 2. 초기화 함수 ---
    async def initialize(self):
        """UI 요소가 이미 로드된 후 ROS 클라이언트 초기화 및 런치 파일 검색 시작"""
        
        # 1. ROS 클라이언트 설정
        self.ros_client = self.setup_ros_client()
        
        # 2. 런치 파일 검색 및 UI 렌더링
        dynamic_packages = self.get_workspace_packages()
        all_packages = list(set(DEFAULT_PACKAGES + dynamic_packages)) # 중복 제거
        
        # NOTE: 런치 파일 검색은 시간이 걸릴 수 있으므로, UI 업데이트 전에 실행합니다.
        self.launch_files_data = self.find_launch_files(all_packages)
        self.render_launch_list()
        
        # 3. 상태 업데이트 타이머 시작
        ui.timer(0.5, self.poll_ros_and_update_ui)
        
    def get_workspace_packages(self) -> List[str]:
        src_path = os.path.join(ROS_WORKSPACE_PATH, 'src')
        package_list = []
        
        if not os.path.isdir(src_path):
            print(f"경고: 소스 디렉터리를 찾을 수 없습니다: {src_path}")
            return package_list

        try:
            for item in os.listdir(src_path):
                package_path = os.path.join(src_path, item)
                if os.path.isdir(package_path) and os.path.exists(os.path.join(package_path, 'package.xml')):
                    package_list.append(item)
        except Exception as e:
            print(f"워크스페이스 패키지 검색 오류: {e}")
            
        return package_list


    def setup_ros_client(self):
        try:
            # roslibpy는 NiceGUI의 이벤트 루프와 독립적으로 동작하므로 여기서 생성합니다.
            import roslibpy
            client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
            client.run_in_thread()
            return client
        except ImportError:
            if self.status_bar_lbl:
                 self.status_bar_lbl.set_text("FATAL: roslibpy 패키지 없음").classes(replace="text-xs text-red-700 font-bold")
            return None
        except Exception:
            return None

    def poll_ros_and_update_ui(self):
        if not self.status_bar_lbl or not self.ros_client:
            return

        is_connected = self.ros_client.is_connected
        
        if is_connected and not self.ros_connected:
            self.ros_connected = True
            self.status_bar_lbl.set_text("🟢 ROS Bridge 연결됨").classes(replace="text-xs text-green-500 font-bold")
            
        elif not is_connected and self.ros_connected:
            self.ros_connected = False
            self.status_bar_lbl.set_text("❌ ROS Bridge 연결 끊김").classes(replace="text-xs text-red-500 font-bold")

        elif not is_connected and not self.ros_connected:
            self.ros_connected = False
            self.status_bar_lbl.set_text("❌ ROS Bridge 연결 비활성").classes(replace="text-xs text-gray-700 font-bold")


    def find_launch_files(self, packages: List[str]) -> Dict[str, Any]:
        """주어진 ROS 패키지 목록에서 런치 파일을 검색합니다."""
        launch_data = {}
        # print(f"INFO: 런치 파일 검색 대상 패키지: {packages}") # 터미널 출력 최소화
        
        try:
            for pkg in packages:
                result = subprocess.run(
                    f"{ROS_SETUP_COMMAND} && rospack find {pkg}",
                    shell=True, capture_output=True, text=True, executable='/bin/bash'
                )
                pkg_path = result.stdout.strip()
                
                if pkg_path and os.path.isdir(pkg_path):
                    launch_path = os.path.join(pkg_path, 'launch')
                    if os.path.isdir(launch_path):
                        launch_files = [
                            f for f in os.listdir(launch_path) 
                            if f.endswith(('.launch', '.xml', '.py'))
                        ]
                        if launch_files:
                            launch_data[pkg] = {
                                'path': pkg_path,
                                'files': {
                                    f: {'status': 'STOPPED', 'process': None} for f in launch_files
                                }
                            }
        except Exception as e:
            print(f"Error finding launch files: {e}")
            ui.notify(f"런치 파일 검색 오류: {e}", type='negative')
            
        return launch_data

    # --- 런치 파일 실행/정지 로직 ---
    def toggle_launch(self, pkg_name: str, file_name: str, file_button: ui.button):
        file_info = self.launch_files_data[pkg_name]['files'][file_name]
        
        if file_info['status'] == 'STOPPED':
            self.run_launch(pkg_name, file_name, file_info, file_button)
        else:
            self.stop_launch(file_name, file_info, file_button)

    def run_launch(self, pkg_name, file_name, file_info, file_button):
        try:
            launch_command = f"{ROS_SETUP_COMMAND} && roslaunch {pkg_name} {file_name}"
            
            process = subprocess.Popen(
                launch_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                executable='/bin/bash'
            )
            
            file_info['status'] = 'RUNNING'
            file_info['process'] = process
            
            # 버튼 업데이트 (실행 중 상태)
            file_button.set_text('RUNNING')
            file_button.props('color=red icon=pause')
            # 람다 함수 내에서 self에 접근하기 위해 클래스 인스턴스를 직접 사용합니다.
            file_button.on('click', lambda: NiceGUIRos_instance.toggle_launch(pkg_name, file_name, file_button)) 
            ui.notify(f"런치 파일 '{file_name}' 실행 시작", type='positive')

        except Exception as e:
            ui.notify(f"런치 파일 실행 오류: {e}", type='negative')
            file_info['status'] = 'STOPPED'
            
            if file_button:
                file_button.set_text('RUN')
                file_button.props('color=green icon=play_arrow')

    def stop_launch(self, file_name, file_info, file_button):
        process = file_info.get('process')
        if process:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                
            file_info['status'] = 'STOPPED'
            file_info['process'] = None
            
            if file_button:
                file_button.set_text('RUN')
                file_button.props('color=green icon=play_arrow')
                # 람다 함수 내에서 self에 접근하기 위해 클래스 인스턴스를 직접 사용합니다.
                file_button.on('click', lambda name=file_name, pkg=file_info['pkg_name'], btn=file_button: 
                                    NiceGUIRos_instance.toggle_launch(pkg, name, btn)
                                )
            ui.notify(f"런치 파일 '{file_name}' 정지됨", type='info')
            
    def stop_all_launches(self):
        # 모든 실행 중인 프로세스 정지 로직 
        # TODO: 현재 실행 중인 모든 런치 파일을 순회하며 stop_launch를 호출합니다.
        pass 

    # --- UI 렌더링 ---
    def render_launch_list(self):
        if not self.launch_list_container: return

        self.launch_list_container.clear()
        
        if not self.launch_files_data:
            with self.launch_list_container:
                ui.label("⚠️ ROS 패키지가 없거나, 런치 파일이 없습니다.")\
                    .classes('text-orange-500 p-4 bg-orange-100 rounded-lg')
            return

        with self.launch_list_container:
            for pkg_name, pkg_data in self.launch_files_data.items():
                ui.label(pkg_name).classes('text-xl font-bold mt-4 mb-2 text-gray-800')
                
                with ui.column().classes('w-full border p-2 rounded-lg shadow-sm'):
                    for file_name, file_info in pkg_data['files'].items():
                        
                        with ui.row().classes('w-full items-center justify-between py-1 px-2 hover:bg-gray-50 rounded'):
                            
                            with ui.column().classes('p-0 m-0'):
                                ui.label(file_name).classes('text-base font-medium')
                                ui.label(f"Package: {pkg_name}").classes('text-xs text-gray-500')
                            
                            status = file_info['status']
                            color = 'green' if status == 'STOPPED' else 'red'
                            icon = 'play_arrow' if status == 'STOPPED' else 'pause'
                            
                            file_button = ui.button(status, color=color, icon=icon).classes('w-24')
                            
                            file_info['pkg_name'] = pkg_name 
                            
                            # 콜백 함수에서 클래스 인스턴스(NiceGUIRos_instance)를 사용하도록 변경
                            file_button.on('click', lambda name=file_name, pkg=pkg_name, btn=file_button: 
                                NiceGUIRos_instance.toggle_launch(pkg, name, btn)
                            )


# ----------------------------------------------------
# 3. 페이지 정의 및 서버 시작 블록
# ----------------------------------------------------

# 클래스 인스턴스를 전역으로 생성 (페이지 함수가 접근할 수 있도록)
NiceGUIRos_instance = NiceGUIRos() 

@ui.page('/')
def main_page():
    """웹 페이지의 루트 경로에 UI를 구성하는 함수."""
    global NiceGUIRos_instance
    
    # 헤더
    with ui.header().classes('items-center shadow-lg').style('background-color: #2c3e50'):
        ui.label('WeGo').classes('text-lg font-bold text-white')
        ui.label('MINI π ROS Launch Manager').classes('ml-4 text-white')
        ui.space()
        ui.button('모두 정지', color='red', on_click=NiceGUIRos_instance.stop_all_launches).classes('shadow-md')

    with ui.row().classes('w-full'):
        with ui.column().classes('w-full'):
            with ui.tabs().classes('w-full') as tabs:
                ui.tab('A. 런치 파일 목록').classes('font-bold')
                ui.tab('B. 로그').classes('font-bold')
                ui.tab('C. 영상 토픽').classes('font-bold')

            with ui.tab_panels(tabs, value='A. 런치 파일 목록').classes('w-full p-4'):
                with ui.tab_panel('A. 런치 파일 목록').classes('p-0'):
                    ui.input(placeholder='패키지/런치 파일 검색').classes('w-full mb-4')
                    
                    # UI 요소를 클래스 인스턴스에 할당
                    NiceGUIRos_instance.launch_list_container = ui.column().classes('w-full')
                    with NiceGUIRos_instance.launch_list_container:
                        ui.label("런치 파일 목록 로딩 중...")

                with ui.tab_panel('B. 로그'):
                    ui.label('선택된 런치 파일의 로그가 표시됩니다.')
                    ui.card().classes('w-full h-96 bg-gray-900 text-white font-mono p-2 overflow-auto')
                    
                with ui.tab_panel('C. 영상 토픽'):
                    ui.label('영상 토픽 스트림이 표시됩니다.')
                    ui.label("이곳에 ROS 영상 스트림이 표시될 예정입니다.").classes('w-full h-96 flex items-center justify-center border-dashed border-2')
                    
    # 상태 바 (하단)
    with ui.footer().classes('justify-start items-center h-6 bg-gray-100 shadow-inner'):
         # UI 요소를 클래스 인스턴스에 할당
         NiceGUIRos_instance.status_bar_lbl = ui.label("ROS Bridge 연결 초기화 중").classes("text-xs text-gray-700 font-bold")

    # UI 구성이 완료된 직후 초기화 작업 시작
    ui.timer(0.5, NiceGUIRos_instance.initialize, once=True)


# NiceGUI가 요구하는 멀티프로세싱 가드를 사용
if __name__ in {"__main__", "__mp_main__"}:
    # app.on_startup 대신 @ui.page를 사용하여 안정적으로 페이지를 구성합니다.
    # ui.run()을 호출하여 서버를 즉시 시작합니다. (포트 8088 사용, 라이트 모드)
    ui.run(host='0.0.0.0', port=8088, title='ROS Launch Manager', reload=False)