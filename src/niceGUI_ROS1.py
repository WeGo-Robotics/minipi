#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import signal
import time
import threading
from typing import Dict, Any, Optional, List

from nicegui import ui, app

# --- 1. ROS 환경 설정 ---
# ROS Bridge 호스트는 로봇 PC의 실제 IP 주소입니다. (192.168.0.48)
ROSBRIDGE_HOST = "192.168.0.48"
ROSBRIDGE_PORT = 9090

# ROS 1의 워크스페이스 소싱 명령 (실제 환경에서 사용자가 설정한 값)
ROS_SETUP_COMMAND = os.environ.get('ROS_SETUP_COMMAND', "source /opt/ros/noetic/setup.bash && source ~/soccer_ws/devel/setup.bash")

# Launch 파일을 검색할 ROS 패키지 목록
# 실제 사용할 패키지로 대체하거나, 환경 변수에서 가져오도록 수정 가능
LAUNCH_PACKAGES = ['rviz_visual_tools', 'turtlesim', 'usb_cam'] 

class NiceGUIRos:
    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.status_bar_lbl: Optional[ui.label] = None
        
        # ROS 연결 상태 및 클라이언트
        self.ros_connected = False
        self.ros_client = self.setup_ros_client() 
        
        # UI 빌드 및 상태 레이블 초기화
        self.build_ui()
        
        # 런치 파일 검색
        self.launch_files_data = self.find_launch_files(LAUNCH_PACKAGES)
        self.render_launch_list()
        
        # 타이머 실행 지연 (UI 요소 로드 후 상태 업데이트 시작)
        ui.timer(1.0, self.start_ros_timer, once=True) 
        
    def setup_ros_client(self):
        try:
            import roslibpy
            # roslibpy.Ros 객체를 생성하고 백그라운드 스레드에서 연결 시도를 시작합니다.
            client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
            client.run_in_thread()
            return client
        except ImportError:
            ui.notify("roslibpy 패키지가 설치되지 않았습니다.", type='negative')
            return None
        except Exception:
            # 초기 연결 실패는 타이머가 처리하므로 무시
            return None

    def start_ros_timer(self):
        # 0.5초마다 ROS 상태를 확인하고 UI를 업데이트합니다.
        ui.timer(0.5, self.poll_ros_and_update_ui)
        
    def poll_ros_and_update_ui(self):
        if not self.status_bar_lbl or not self.ros_client:
            return

        is_connected = self.ros_client.is_connected
        
        if is_connected and not self.ros_connected:
            self.ros_connected = True
            # 연결 성공 시 초록색 표시
            self.status_bar_lbl.set_text("🟢 ROS Bridge 연결됨").classes(replace="text-xs text-green-500 font-bold")
            
        elif not is_connected and self.ros_connected:
            self.ros_connected = False
            # 연결 끊김 시 빨간색 표시
            self.status_bar_lbl.set_text("❌ ROS Bridge 연결 끊김").classes(replace="text-xs text-red-500 font-bold")

        elif not is_connected and not self.ros_connected:
            # 연결 비활성 시 회색 표시
            self.status_bar_lbl.set_text("❌ ROS Bridge 연결 비활성").classes(replace="text-xs text-gray-700 font-bold")

    def find_launch_files(self, packages: List[str]) -> Dict[str, Any]:
        launch_data = {}
        try:
            # find 명령어를 사용하여 패키지 내부의 launch 파일을 검색 (ROS 1 방식)
            for pkg in packages:
                # ROS 패키지 경로 찾기 (catkin_find는 ROS 1에서 주로 사용)
                result = subprocess.run(
                    f"source /opt/ros/noetic/setup.bash && rospack find {pkg}",
                    shell=True, capture_output=True, text=True, executable='/bin/bash'
                )
                pkg_path = result.stdout.strip()
                
                if pkg_path:
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
            # roslauch 명령을 쉘로 실행하고 프로세스 객체 저장
            launch_command = f"{ROS_SETUP_COMMAND} && roslaunch {pkg_name} {file_name}"
            
            # Popen을 사용하여 새로운 터미널 세션 없이 백그라운드에서 실행
            # preexec_fn=os.setsid는 프로세스 그룹을 만들어 한 번에 종료하기 위함 (stop_launch 참고)
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
            file_button.set_text('RUNNING').props('color=red icon=pause').on('click', lambda: self.toggle_launch(pkg_name, file_name, file_button))
            ui.notify(f"런치 파일 '{file_name}' 실행 시작", type='positive')

        except Exception as e:
            ui.notify(f"런치 파일 실행 오류: {e}", type='negative')
            file_info['status'] = 'STOPPED'
            file_button.set_text('STOPPED').props('color=green icon=play_arrow')

    def stop_launch(self, file_name, file_info, file_button):
        process = file_info.get('process')
        if process:
            try:
                # 프로세스 그룹 전체에 SIGTERM 시그널을 보냄 (roslaunch 종료에 효과적)
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5) # 5초 대기
                
            except ProcessLookupError:
                # 프로세스가 이미 종료되었을 수 있음
                pass
            except subprocess.TimeoutExpired:
                # 5초 후에도 종료되지 않으면 SIGKILL
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                
            file_info['status'] = 'STOPPED'
            file_info['process'] = None
            file_button.set_text('RUN').props('color=green icon=play_arrow').on('click', lambda: self.toggle_launch(file_info['pkg_name'], file_name, file_button))
            ui.notify(f"런치 파일 '{file_name}' 정지됨", type='info')
            
    def stop_all_launches(self):
        # 실행 중인 모든 런치 파일을 정지하는 로직 (생략)
        pass # UI에서 '모두 정지' 버튼의 on_click에 연결될 예정

    # --- UI 렌더링 ---
    def render_launch_list(self):
        """A. 패키지 ON/OFF 탭에 런치 파일 목록을 렌더링합니다."""
        
        # 런치 파일 목록을 렌더링할 컨테이너를 찾습니다.
        if not hasattr(self, 'launch_list_container'):
             return

        self.launch_list_container.clear()
        
        if not self.launch_files_data:
            with self.launch_list_container:
                ui.label("⚠️ ROS 패키지가 없거나, 환경 변수 LAUNCH_PACKAGES 설정에 맞는 런치 파일이 없습니다.")\
                    .classes('text-orange-500 p-4 bg-orange-100 rounded-lg')
            return

        with self.launch_list_container:
            for pkg_name, pkg_data in self.launch_files_data.items():
                # 패키지 이름 헤더
                ui.label(pkg_name).classes('text-xl font-bold mt-4 mb-2 text-gray-800')
                
                with ui.column().classes('w-full border p-2 rounded-lg shadow-sm'):
                    for file_name, file_info in pkg_data['files'].items():
                        
                        # 파일 이름과 버튼을 포함하는 행
                        with ui.row().classes('w-full items-center justify-between py-1 px-2 hover:bg-gray-50 rounded'):
                            
                            # 좌측: 파일 정보
                            with ui.column().classes('p-0 m-0'):
                                ui.label(file_name).classes('text-base font-medium')
                                ui.label(f"Package: {pkg_name}").classes('text-xs text-gray-500')
                            
                            # 우측: RUN/STOP 버튼
                            status = file_info['status']
                            color = 'green' if status == 'STOPPED' else 'red'
                            icon = 'play_arrow' if status == 'STOPPED' else 'pause'
                            
                            # ui.button을 먼저 생성합니다.
                            file_button = ui.button(status, color=color, icon=icon).classes('w-24')
                            
                            # 런치 파일 정보를 버튼에 연결하여 콜백 함수가 사용할 수 있도록 합니다.
                            file_info['pkg_name'] = pkg_name 
                            
                            # 버튼 클릭 시 toggle_launch 호출
                            file_button.on('click', lambda name=file_name, pkg=pkg_name, btn=file_button: 
                                self.toggle_launch(pkg, name, btn)
                            )


    def build_ui(self):
        ui.page_title('WeGo MINI π ROS Launch Manager')
        
        # 헤더
        with ui.header().classes('items-center shadow-lg').style('background-color: #2c3e50'):
            ui.label('WeGo').classes('text-lg font-bold text-white')
            ui.label('MINI π ROS Launch Manager').classes('ml-4 text-white')
            ui.space()
            ui.button('모두 정지', color='red', on_click=self.stop_all_launches).classes('shadow-md')

        with ui.row().classes('w-full'):
            with ui.column().classes('w-full'):
                # 탭 컨테이너
                with ui.tabs().classes('w-full') as tabs:
                    ui.tab('A. 런치 파일 목록').classes('font-bold')
                    ui.tab('B. 로그').classes('font-bold')
                    ui.tab('C. 영상 토픽').classes('font-bold')

                # 탭 패널
                with ui.tab_panels(tabs, value='A. 런치 파일 목록').classes('w-full p-4'):
                    with ui.tab_panel('A. 런치 파일 목록').classes('p-0'):
                        ui.input(placeholder='패키지/런치 파일 검색').classes('w-full mb-4')
                        # 런치 파일 목록이 렌더링될 컨테이너
                        self.launch_list_container = ui.column().classes('w-full')
                        # 초기 로딩 시 메시지를 표시 (render_launch_list에서 대체될 예정)
                        with self.launch_list_container:
                            ui.label("런치 파일 목록 로딩 중...")

                    with ui.tab_panel('B. 로그'):
                        ui.label('선택된 런치 파일의 로그가 표시됩니다.')
                        ui.card().classes('w-full h-96 bg-gray-900 text-white font-mono p-2 overflow-auto') # 로그 표시 영역
                        
                    with ui.tab_panel('C. 영상 토픽'):
                        ui.label('영상 토픽 스트림이 표시됩니다.')
                        ui.label("이곳에 ROS 영상 스트림이 표시될 예정입니다.").classes('w-full h-96 flex items-center justify-center border-dashed border-2')
                        
        # 상태 바 (하단)
        with ui.footer().classes('justify-start items-center h-6 bg-gray-100 shadow-inner'):
             # self.status_bar_lbl을 초기화합니다.
             self.status_bar_lbl = ui.label("ROS Bridge 연결 초기화 중").classes("text-xs text-gray-700 font-bold")


# NiceGUI가 요구하는 멀티프로세싱 가드를 사용
if __name__ in {"__main__", "__mp_main__"}:
    # NiceGUIRos 클래스의 인스턴스를 ui.run() 호출 전에 생성하여 UI와 ROS를 초기화합니다.
    NiceGUIRos() 
    
    # ui.run()을 호출하여 서버를 시작합니다.
    ui.run(host='0.0.0.0', port=8088, title='ROS Launch Manager', reload=False)