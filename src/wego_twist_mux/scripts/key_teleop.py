#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QGridLayout, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# ROS 메시지 임포트 및 필드 정의
try:
    from geometry_msgs.msg import Twist
    from sim2real_msg.msg import ControlState
    CONTROL_STATE_FIELDS = [
        'standby', 'sitdown', 'running_standby_switch', 'pitch_ctrl', 
        'confirm', 'quit', 'hip_angle_increase', 'hip_angle_decrease'
    ]
except ImportError:
    rospy.logwarn("Missing ROS Message types. Using dummy classes.")
    
    # Twist 더미 클래스
    class Twist:
        def __init__(self):
            self.linear = type('L', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            self.angular = type('A', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            
    # ControlState 더미 클래스
    class ControlState:
        def __init__(self):
            self.candidate_left = 0.0
            self.standby = 0.0
            self.sitdown = 0.0
            self.running_standby_switch = 0.0
            self.pitch_ctrl = 0.0
            self.confirm = 0.0
            self.quit = 0.0
            self.hip_angle_increase = 0.0
            self.hip_angle_decrease = 0.0
    CONTROL_STATE_FIELDS = ['standby', 'sitdown', 'running_standby_switch', 'pitch_ctrl', 'confirm', 'quit']


# -------------------------------------------------------
# 1. ROS Publisher 클래스: Twist (이동)
# -------------------------------------------------------
class TwistPublisher:
    def __init__(self, topic_name="cmd_vel/joy"):
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=1)
        self.twist_msg = Twist()
        
        # 조이스틱 설정에 따른 스케일 값
        self.linear_x_scale = 0.6
        self.linear_y_scale = 0.7
        self.angular_z_scale = 0.7
        
        # WASD/QE 키 매핑: (linear.x, linear.y, angular.z)
        self.key_bindings = {
            'w': (self.linear_x_scale, 0.0, 0.0),
            's': (-self.linear_x_scale, 0.0, 0.0),
            'a': (0.0, self.linear_y_scale, 0.0), # linear.y (스트레이핑)
            'd': (0.0, -self.linear_y_scale, 0.0),
            'q': (0.0, 0.0, self.angular_z_scale), # angular.z (회전)
            'e': (0.0, 0.0, -self.angular_z_scale),
        }

    def publish_command(self, key):
        """키 입력에 따라 Twist 메시지를 설정하고 발행합니다."""
        if key in self.key_bindings:
            vx, vy, vz = self.key_bindings[key]
            
            self.twist_msg.linear.x = vx
            self.twist_msg.linear.y = vy
            self.twist_msg.angular.z = vz
            
            self.twist_msg.linear.z = 0.0 
            self.twist_msg.angular.x = 0.0
            self.twist_msg.angular.y = 0.0
            
            self.pub.publish(self.twist_msg)
            return True
        return False

    def stop(self):
        """로봇을 정지시키고 메시지를 발행합니다."""
        # Twist 메시지의 속도가 0이 아닐 경우에만 발행
        if (self.twist_msg.linear.x != 0.0 or 
            self.twist_msg.linear.y != 0.0 or 
            self.twist_msg.angular.z != 0.0):
            
            self.twist_msg.linear.x = 0.0
            self.twist_msg.linear.y = 0.0
            self.twist_msg.angular.z = 0.0
            self.pub.publish(self.twist_msg)

    def get_current_values(self):
        """현재 발행된 Twist 값을 반환합니다."""
        return {
            'linear.x': self.twist_msg.linear.x, 
            'linear.y': self.twist_msg.linear.y, 
            'angular.z': self.twist_msg.angular.z
        }

# -------------------------------------------------------
# 2. ROS Publisher 클래스: ControlState (상태)
# -------------------------------------------------------
class ControlStatePublisher:
    def __init__(self, topic_name="/joy_msg"):
        self.pub = rospy.Publisher(topic_name, ControlState, queue_size=1)
        self.active_fields = {}
        self.last_published_msg = self._create_zero_message()
        
        self.key_mappings = {
            '1': ('sitdown', 1.0),
            '2': ('standby', 1.0),
            '3': ('running_standby_switch', 1.0),
            '4': ('confirm', 1.0),
            '5': ('quit', 1.0),
            'z': ('pitch_ctrl', 1.0), 
            'x': ('pitch_ctrl', -1.0),
        }

        self.timer = QTimer()
        self.timer.timeout.connect(self._publish_current_state)
        
        # ROS 노드 초기화가 메인 블록으로 이동했기 때문에 안전하게 호출 가능
        self._publish_zero_message() 

    def _create_zero_message(self):
        """모든 필드가 0.0으로 초기화된 새 메시지 객체를 생성합니다."""
        return ControlState()

    def _publish_current_state(self):
        """현재 active_fields를 기반으로 메시지를 생성하고 발행합니다."""
        if rospy.is_shutdown():
            self.timer.stop()
            return

        current_msg = self._create_zero_message()
        for field, value in self.active_fields.items():
            if hasattr(current_msg, field):
                setattr(current_msg, field, value)
        
        self.pub.publish(current_msg)
        self.last_published_msg = current_msg

    def handle_key_press(self, key):
        """키보드 누름 이벤트 처리."""
        if key in self.key_mappings:
            field, value = self.key_mappings[key]
            
            self.active_fields.clear() 
            self.active_fields[field] = value
            
            self._publish_current_state()
            if not self.timer.isActive():
                self.timer.start(50)
            return True
        return False

    def handle_key_release(self, key):
        """키보드 떼기 이벤트 처리."""
        if key in self.key_mappings:
            field_name, _ = self.key_mappings[key]
            
            if field_name in self.active_fields:
                self.active_fields.pop(field_name)
                
                if not self.active_fields:
                    self.timer.stop()
                    self._publish_zero_message()
                else:
                    self._publish_current_state() 
                return True
        return False

    def _publish_zero_message(self):
        zero_msg = self._create_zero_message()
        self.pub.publish(zero_msg)
        self.last_published_msg = zero_msg

    def get_current_values(self):
        """ROS 메시지 필드를 수동으로 추출하여 딕셔너리로 반환합니다."""
        msg = self.last_published_msg
        current_values = {}
        for field in CONTROL_STATE_FIELDS:
            if hasattr(msg, field):
                current_values[field] = getattr(msg, field)
        return current_values

# -------------------------------------------------------
# 3. PyQt GUI 클래스
# -------------------------------------------------------
class TeleopGUI(QWidget):
    def __init__(self, twist_pub, cs_pub):
        super().__init__()
        self.twist_pub = twist_pub
        self.cs_pub = cs_pub
        
        # 키보드 상태 추적
        self.active_twist_keys = set()
        
        self.setWindowTitle('ROS Teleop Control (Twist & ControlState)')
        self.initUI()
        self.setFocusPolicy(Qt.StrongFocus)
        
        # GUI 업데이트 타이머
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_values)
        self.timer.start(100)

    def initUI(self):
        vbox = QVBoxLayout()
        header_font = QFont("Arial", 14, QFont.Bold)
        value_font = QFont("Courier New", 12)
        
        # 1. Twist (이동) 제어 섹션
        twist_box = QGroupBox("1. Movement Control (cmd_vel/joy)")
        twist_layout = QGridLayout()
        self.twist_labels = {}
        twist_items = [
            ("Linear X (W/S)", 'linear.x'),
            ("Linear Y (A/D)", 'linear.y'),
            ("Angular Z (Q/E)", 'angular.z')
        ]
        
        for i, (text, key) in enumerate(twist_items):
            twist_layout.addWidget(QLabel(text + ":"), i, 0, alignment=Qt.AlignRight)
            label = QLabel("0.00")
            label.setFont(value_font)
            label.setStyleSheet("color: blue;")
            self.twist_labels[key] = label
            twist_layout.addWidget(label, i, 1, alignment=Qt.AlignLeft)
        
        twist_box.setLayout(twist_layout)
        vbox.addWidget(twist_box)
        
        # 2. ControlState (상태) 제어 섹션
        cs_box = QGroupBox("2. State Control (/joy_msg)")
        cs_layout = QGridLayout()
        self.cs_labels = {}
        
        cs_items = [
            ("SITDOWN (1)", 'sitdown'), 
            ("STANDBY (2)", 'standby'),
            ("RUNNING_SWITCH (3)", 'running_standby_switch'), 
            ("PITCH_CTRL (Z/X)", 'pitch_ctrl'),
        ]
        
        for i, (text, key) in enumerate(cs_items):
            cs_layout.addWidget(QLabel(text + ":"), i, 0, alignment=Qt.AlignRight)
            label = QLabel("0.0")
            label.setFont(value_font)
            label.setStyleSheet("color: blue;")
            self.cs_labels[key] = label
            cs_layout.addWidget(label, i, 1, alignment=Qt.AlignLeft)
            
        cs_box.setLayout(cs_layout)
        vbox.addWidget(cs_box)
        
        # 3. 키 가이드
        vbox.addSpacing(10)
        vbox.addWidget(QLabel("--- Key Guide ---", font=header_font), alignment=Qt.AlignCenter)
        guide = QLabel(
            "Movement: W/S (X), A/D (Y), Q/E (Z-Rot)\n"
            "State: 1(Sit), 2(Stand), 3(Switch), Z/X(Pitch)\n"
            "STOP: SPACE (Twist) | Key Release (ControlState)"
        )
        vbox.addWidget(guide, alignment=Qt.AlignCenter)
        
        vbox.addStretch(1)
        self.setLayout(vbox)
        self.show()

    def update_values(self):
        """GUI의 값을 주기적으로 업데이트합니다."""
        # Twist 업데이트
        twist_values = self.twist_pub.get_current_values()
        for key, value in twist_values.items():
            if key in self.twist_labels:
                self.twist_labels[key].setText(f"{value:.2f}")
                self.twist_labels[key].setStyleSheet("color: red;" if abs(value) > 0.0 else "color: blue;")

        # ControlState 업데이트
        cs_values = self.cs_pub.get_current_values()
        for key, value in cs_values.items():
            if key in self.cs_labels:
                # 버튼(0.0/1.0)
                if key in ['sitdown', 'standby', 'running_standby_switch']:
                    self.cs_labels[key].setText(f"{int(value)}")
                    self.cs_labels[key].setStyleSheet("color: red;" if value == 1.0 else "color: blue;")
                # Axis(실수)
                else:
                    self.cs_labels[key].setText(f"{value:.2f}")
                    self.cs_labels[key].setStyleSheet("color: red;" if abs(value) > 0.0 else "color: blue;")

    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return

        key_char = event.text().lower()
        key_code = event.key()
        
        # Twist (Movement) 키 처리
        if key_char in self.twist_pub.key_bindings:
            self.twist_pub.publish_command(key_char)
            self.active_twist_keys.add(key_char) # 키 상태 추적
            self.update_values()
        
        # Twist 정지 키 처리
        elif key_code == Qt.Key_Space:
            self.twist_pub.stop()
            self.active_twist_keys.clear()
            self.update_values()
            
        # ControlState (State) 키 처리 (숫자 키 포함)
        elif key_code == Qt.Key_1: key_char = '1'
        elif key_code == Qt.Key_2: key_char = '2'
        elif key_code == Qt.Key_3: key_char = '3'
        elif key_code == Qt.Key_4: key_char = '4'
        elif key_code == Qt.Key_5: key_char = '5'
            
        if self.cs_pub.handle_key_press(key_char):
            self.update_values()
            
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
            
        key_char = event.text().lower()
        key_code = event.key()

        # Twist (Movement) 키 처리
        if key_char in self.twist_pub.key_bindings:
            self.active_twist_keys.discard(key_char)
            # 다른 이동 키가 눌려있지 않고, Spacebar가 아니라면 정지 명령 발행
            if not self.active_twist_keys:
                self.twist_pub.stop()
                self.update_values()
        
        # ControlState (State) 키 처리 (숫자 키 포함)
        elif key_code == Qt.Key_1: key_char = '1'
        elif key_code == Qt.Key_2: key_char = '2'
        elif key_code == Qt.Key_3: key_char = '3'
        elif key_code == Qt.Key_4: key_char = '4'
        elif key_code == Qt.Key_5: key_char = '5'
            
        if self.cs_pub.handle_key_release(key_char):
            self.update_values()

# -------------------------------------------------------
# 4. 메인 실행 함수
# -------------------------------------------------------
if __name__ == '__main__':
    # [핵심 수정 부분]: ROS 노드 초기화를 가장 먼저 수행합니다.
    try:
        rospy.init_node('keyboard_teleop_combined', anonymous=True)
    except rospy.exceptions.ROSException:
        pass # 이미 초기화된 경우 무시
        
    app = QApplication(sys.argv)
    
    # ROS 퍼블리셔 인스턴스 생성 (이제 init_node() 이후에 생성되므로 안전)
    twist_pub = TwistPublisher()
    cs_pub = ControlStatePublisher()
    
    # PyQt GUI 인스턴스 생성 및 두 퍼블리셔 연결
    ex = TeleopGUI(twist_pub, cs_pub)
    
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")