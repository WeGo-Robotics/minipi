#!/usr/bin/env python3
import rospy
from nicegui import ui
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from web_gui_pkg.core_logic import NiceGUIRos_instance

class ControlPage:
    def __init__(self):
        # 상태 변수 초기화
        self.current_voltage = 0.0
        
        # [핵심 수정] ROS 노드가 아직 초기화되지 않았다면 초기화 수행
        # disable_signals=True는 NiceGUI 서버 종료(Ctrl+C)와 충돌을 막기 위함
        if not rospy.core.is_initialized():
            rospy.init_node('web_gui_control_node', anonymous=True, disable_signals=True)
        
        # ROS Publisher & Subscriber 설정
        self.pub = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=10)
        self.sub_bat = rospy.Subscriber('/battery_voltage', Float32, self.battery_callback)

    def battery_callback(self, msg):
        self.current_voltage = msg.data

    def publish_vel(self, linear, angular):
        # Publisher가 없으면 실행하지 않음
        if not hasattr(self, 'pub'): return
        
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def stop_vel(self):
        self.publish_vel(0.0, 0.0)

    def create_page(self):
        @ui.page('/control')
        def control_view():
            # --- 1. 헤더 ---
            with ui.header().classes('bg-[#0f172a] text-white flex justify-between items-center p-4'):
                with ui.row().classes('items-center gap-2'):
                    with ui.element('div').classes('w-8 h-8 bg-blue-500 rounded-full flex items-center justify-center'):
                        ui.icon('token', color='white', size='xs')
                    ui.label('Mini Pi ROS Launch Manager').classes('text-lg font-bold')
                
                # 모두 정지 버튼
                ui.button('■ 모두 정지', on_click=lambda: NiceGUIRos_instance.stop_all_launches())\
                    .classes('bg-red-500 text-white font-bold px-4 py-2 rounded hover:bg-red-600 border-none')

            # --- 2. 탭 네비게이션 ---
            with ui.row().classes('w-full justify-center bg-white border-b border-gray-200 py-4 mb-6'):
                ui.link('LAUNCHES', '/').classes('px-6 py-2 text-gray-500 font-medium no-underline hover:text-black')
                ui.link('LOGS', '#').classes('px-6 py-2 text-gray-500 font-medium no-underline hover:text-black')
                ui.link('VIDEO', '#').classes('px-6 py-2 text-gray-500 font-medium no-underline hover:text-black')
                ui.link('CONTROL', '/control').classes('px-6 py-2 text-[#0f172a] border-b-4 border-[#0f172a] font-bold no-underline')

            # --- 3. 메인 컨텐츠 ---
            with ui.row().classes('w-full max-w-7xl mx-auto gap-6 p-4 items-start'):
                
                # [왼쪽] 카메라 뷰
                with ui.card().classes('flex-grow bg-black h-[600px] relative p-0 flex items-center justify-center overflow-hidden rounded-lg shadow-lg'):
                    with ui.row().classes('absolute top-4 left-4 right-4 justify-between z-10'):
                        ui.label('Camera View').classes('text-white bg-black/50 px-2 py-1 rounded')
                        ui.badge('Live', color='red').props('outline')

                    ui.image('/video_feed').classes('w-full h-full object-contain')

                # [오른쪽] 제어 및 배터리
                with ui.column().classes('w-full md:w-[350px] gap-6'):
                    
                    # (1) 이동 제어
                    with ui.card().classes('w-full bg-[#2d2d2d] p-6 items-center text-center rounded-lg shadow-lg'):
                        ui.label('이동 제어').classes('text-[#60a5fa] text-xl font-bold mb-6')
                        
                        with ui.grid(columns=3).classes('gap-3'):
                            # 스타일 정의
                            btn_style = 'w-20 h-20 bg-gray-600 text-white text-3xl rounded hover:bg-gray-500 border-none'
                            stop_style = 'w-20 h-20 bg-red-600 text-white text-3xl rounded hover:bg-red-500 border-none'

                            # Row 1
                            ui.label('')
                            btn_up = ui.button(icon='arrow_upward').classes(btn_style)
                            btn_up.on('mousedown', lambda: self.publish_vel(0.5, 0.0))
                            btn_up.on('mouseup', self.stop_vel)
                            btn_up.on('mouseleave', self.stop_vel)
                            btn_up.on('touchstart', lambda: self.publish_vel(0.5, 0.0))
                            btn_up.on('touchend', self.stop_vel)
                            ui.label('')
                            
                            # Row 2
                            btn_left = ui.button(icon='arrow_back').classes(btn_style)
                            btn_left.on('mousedown', lambda: self.publish_vel(0.0, 0.5))
                            btn_left.on('mouseup', self.stop_vel)
                            btn_left.on('mouseleave', self.stop_vel)
                            btn_left.on('touchstart', lambda: self.publish_vel(0.0, 0.5))
                            btn_left.on('touchend', self.stop_vel)

                            ui.button(icon='stop', on_click=self.stop_vel).classes(stop_style)

                            btn_right = ui.button(icon='arrow_forward').classes(btn_style)
                            btn_right.on('mousedown', lambda: self.publish_vel(0.0, -0.5))
                            btn_right.on('mouseup', self.stop_vel)
                            btn_right.on('mouseleave', self.stop_vel)
                            btn_right.on('touchstart', lambda: self.publish_vel(0.0, -0.5))
                            btn_right.on('touchend', self.stop_vel)
                            
                            # Row 3
                            ui.label('')
                            btn_down = ui.button(icon='arrow_downward').classes(btn_style)
                            btn_down.on('mousedown', lambda: self.publish_vel(-0.5, 0.0))
                            btn_down.on('mouseup', self.stop_vel)
                            btn_down.on('mouseleave', self.stop_vel)
                            btn_down.on('touchstart', lambda: self.publish_vel(-0.5, 0.0))
                            btn_down.on('touchend', self.stop_vel)
                            ui.label('')

                    # (2) 배터리
                    with ui.card().classes('w-full bg-[#2d2d2d] p-6 items-center text-center rounded-lg shadow-lg'):
                        ui.label('배터리').classes('text-[#60a5fa] text-xl font-bold mb-2')
                        
                        volt_label = ui.label('N/A').classes('text-white text-3xl font-bold mb-4')
                        
                        with ui.element('div').classes('w-full h-8 bg-gray-600 rounded-full overflow-hidden relative'):
                            bar_fill = ui.element('div').classes('h-full bg-green-500 transition-all duration-500')
                            bar_fill.style('width: 0%')

                        ui.label('Max: 25.8V').classes('text-gray-400 text-sm mt-2')

                        def update_battery():
                            v = self.current_voltage
                            volt_label.set_text(f'{v:.2f} V')
                            
                            min_v = 21.0
                            max_v = 25.8
                            
                            if v < 0.1:
                                percent = 0
                            else:
                                percent = ((v - min_v) / (max_v - min_v)) * 100
                                percent = max(0, min(100, percent))
                            
                            bar_fill.style(f'width: {percent}%')
                            
                            if percent < 20:
                                bar_fill.classes('bg-red-500', remove='bg-green-500 bg-yellow-500')
                            elif percent < 50:
                                bar_fill.classes('bg-yellow-500', remove='bg-green-500 bg-red-500')
                            else:
                                bar_fill.classes('bg-green-500', remove='bg-red-500 bg-yellow-500')

                        ui.timer(1.0, update_battery)