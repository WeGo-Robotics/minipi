#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nicegui import ui
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from web_gui_pkg.core_logic import NiceGUIRos_instance


class RemoteControlPage:
    def __init__(self):
        # 배터리 전압 저장
        self.current_voltage = 0.0

        # ROS 노드 초기화 (NiceGUI와 충돌 방지 위해 disable_signals=True)
        if not rospy.core.is_initialized():
            rospy.init_node('web_gui_remote_node', anonymous=True, disable_signals=True)

        # ROS Publisher & Subscriber 설정
        self.pub = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=10)
        self.sub_bat = rospy.Subscriber('/battery_voltage', Float32, self.battery_callback)

    def battery_callback(self, msg: Float32):
        self.current_voltage = msg.data

    def publish_vel(self, linear: float, angular: float):
        if not hasattr(self, 'pub'):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def stop_vel(self):
        self.publish_vel(0.0, 0.0)

    def create_page(self):
        """9090 서버에서 사용할 /remote 페이지 정의"""

        @ui.page('/remote')
        def remote_view():
            # --- 1. 헤더 ---
            with ui.header().classes('bg-[#0f172a] text-white flex justify-between items-center p-4'):
                with ui.row().classes('items-center gap-2'):
                    with ui.element('div').classes(
                        'w-8 h-8 bg-blue-500 rounded-full flex items-center justify-center'
                    ):
                        ui.icon('token', color='white', size='xs')
                    # 제목: Mini Pi Remote
                    ui.label('Mini Pi Remote').classes('text-lg font-bold')

                ui.button(
                    '■ 모두 정지',
                    on_click=lambda: NiceGUIRos_instance.stop_all_launches(),
                ).classes(
                    'bg-red-500 text-white font-bold px-4 py-2 rounded hover:bg-red-600 border-none'
                )

            # --- 2. 메인 레이아웃 ---
            with ui.row().classes('w-full max-w-7xl mx-auto gap-6 p-4 items-start'):

                # [왼쪽] 카메라 + 토픽 선택
                with ui.card().classes(
                    'flex-grow bg-black h-[600px] relative p-0 flex items-center justify-center '
                    'overflow-hidden rounded-lg shadow-lg'
                ):
                    # 상단 오버레이: 제목, Live, 토픽 선택, 구독 버튼
                    with ui.column().classes('absolute top-4 left-4 right-4 gap-2 z-10'):
                        with ui.row().classes('justify-between items-center'):
                            ui.label('Camera View').classes('text-white bg-black/50 px-2 py-1 rounded')
                            ui.badge('Live', color='red').props('outline')

                        with ui.row().classes('items-center gap-2'):
                            topic_select = ui.select(
                                options=[],
                                label='영상 토픽 선택',
                            ).props(
                                # dark + input-class + popup-content-class 로 텍스트 전부 흰색
                                'outlined dense dark input-class=text-white popup-content-class=text-white'
                            ).classes(
                                'w-72 text-white'
                            )

                            # NiceGUIRos에서 이 셀렉트를 사용할 수 있게 연결
                            NiceGUIRos_instance.image_topic_select = topic_select

                            ui.button(
                                '구독 시작',
                                on_click=lambda: NiceGUIRos_instance.subscribe_image_topic(
                                    topic_select.value
                                ),
                            ).classes(
                                'bg-blue-500 text-white px-4 py-2 font-semibold rounded'
                            )

                    # 실제 영상 표시 영역 (NiceGUIRos_instance가 /video_feed URL로 갱신)
                    NiceGUIRos_instance.image_display = ui.image(
                        NiceGUIRos_instance.current_image_src
                    ).classes('w-full h-full object-contain')

                    # 마지막 프레임 정보
                    NiceGUIRos_instance.last_frame_label = ui.label(
                        'Last frame: (none)'
                    ).classes('absolute bottom-4 left-4 text-xs text-gray-400')

                # [오른쪽] 이동 제어 + 배터리
                with ui.column().classes('w-full md:w-[350px] gap-6'):

                    # (1) 이동 제어
                    with ui.card().classes('w-full bg-[#2d2d2d] p-6 items-center text-center rounded-lg shadow-lg'):
                        ui.label('이동 제어').classes('text-[#60a5fa] text-xl font-bold mb-6')

                        with ui.grid(columns=3).classes('gap-3'):
                            btn_style = (
                                'w-20 h-20 bg-gray-600 text-white text-3xl rounded '
                                'hover:bg-gray-500 border-none'
                            )
                            stop_style = (
                                'w-20 h-20 bg-red-600 text-white text-3xl rounded '
                                'hover:bg-red-500 border-none'
                            )

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

                        with ui.element('div').classes(
                            'w-full h-8 bg-gray-600 rounded-full overflow-hidden relative'
                        ):
                            bar_fill = ui.element('div').classes(
                                'h-full bg-green-500 transition-all duration-500'
                            )
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

            # --- 3. 카메라 토픽 / ROS 초기화 ---
            def init_remote():
                try:
                    # 1) core_logic 쪽 공통 초기화 (필요 없는 경우 그냥 놔둬도 무해)
                    NiceGUIRos_instance.initialize()
                except Exception as e:
                    print('[WARN] NiceGUIRos_instance.initialize() 실패:', e)

                try:
                    # 2) ROS1 마스터에서 토픽 목록 직접 가져오기
                    topics = rospy.get_published_topics()
                    # sensor_msgs/Image 또는 CompressedImage 만 필터링
                    image_topics = [
                        name for (name, ttype) in topics
                        if ttype in ('sensor_msgs/Image', 'sensor_msgs/CompressedImage')
                           or 'sensor_msgs/Image' in ttype
                           or 'sensor_msgs/CompressedImage' in ttype
                    ]
                    # 드롭다운 옵션 갱신
                    if hasattr(NiceGUIRos_instance, 'image_topic_select') and \
                            NiceGUIRos_instance.image_topic_select is not None:
                        NiceGUIRos_instance.image_topic_select.options = image_topics
                        print('[INFO] Remote GUI image topics:', image_topics)
                    else:
                        print('[WARN] image_topic_select 가 아직 없음')
                except Exception as e:
                    print('[WARN] ROS 토픽 목록 조회 실패:', e)

            ui.timer(0.5, init_remote, once=True)
