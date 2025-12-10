#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QGridLayout, QGroupBox
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# ==============================================================
#  ROS 메시지 임포트 (Joy 메시지 teleop.yaml 구조로 사용)
# ==============================================================

try:
    from geometry_msgs.msg import Twist
    from sim2real_msg.msg import Joy

    JOY_FIELDS = [
        "l_horizontal",
        "l_vertical",
        "lt",
        "r_horizontal",
        "r_vertical",
        "rt",
        "dpad_horizontal",
        "dpad_vertical",
        "a",
        "b",
        "x",
        "y",
        "lb",
        "rb",
        "back",
        "start",
        "center",
        "L",
        "R",
    ]

except ImportError:
    rospy.logwarn("Missing ROS Message types. Using dummy classes.")

    class Twist:
        def __init__(self):
            self.linear = type("L", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.angular = type("A", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

    class Joy:
        def __init__(self):
            for field in [
                "l_horizontal",
                "l_vertical",
                "lt",
                "r_horizontal",
                "r_vertical",
                "rt",
                "dpad_horizontal",
                "dpad_vertical",
                "a",
                "b",
                "x",
                "y",
                "lb",
                "rb",
                "back",
                "start",
                "center",
                "L",
                "R",
            ]:
                setattr(self, field, 0.0)

    JOY_FIELDS = [
        "l_horizontal",
        "l_vertical",
        "lt",
        "r_horizontal",
        "r_vertical",
        "rt",
        "dpad_horizontal",
        "dpad_vertical",
        "a",
        "b",
        "x",
        "y",
        "lb",
        "rb",
        "back",
        "start",
        "center",
        "L",
        "R",
    ]


# ==============================================================
# 1. Twist Publisher (WASD + QE)
# ==============================================================


class TwistPublisher:
    def __init__(self, topic_name="cmd_vel/auto"):
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=1)
        self.twist_msg = Twist()

        self.key_bindings = {
            "w": (0.6, 0.0, 0.0),
            "s": (-0.6, 0.0, 0.0),
            "a": (0.0, 0.7, 0.0),
            "d": (0.0, -0.7, 0.0),
            "q": (0.0, 0.0, 0.7),
            "e": (0.0, 0.0, -0.7),
        }

    def publish_command(self, key):
        if key not in self.key_bindings:
            return False
        vx, vy, wz = self.key_bindings[key]
        self.twist_msg.linear.x = vx
        self.twist_msg.linear.y = vy
        self.twist_msg.angular.z = wz
        self.pub.publish(self.twist_msg)
        return True

    def stop(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.pub.publish(self.twist_msg)

    def get_current_values(self):
        return {
            "linear.x": self.twist_msg.linear.x,
            "linear.y": self.twist_msg.linear.y,
            "angular.z": self.twist_msg.angular.z,
        }


# ==============================================================
# 2. Joy Publisher (teleop.yaml 구조)
# ==============================================================


class JoyPublisher:
    def __init__(self, topic_name="/joy_msg"):
        self.pub = rospy.Publisher(topic_name, Joy, queue_size=1)

        # 축 값들
        self.axis_values = {
            "l_horizontal": 0.0,
            "l_vertical": 0.0,
            "lt": 0.0,
            "r_horizontal": 0.0,
            "r_vertical": 0.0,
            "rt": 0.0,
            "dpad_horizontal": 0.0,
            "dpad_vertical": 0.0,
        }

        # 버튼 값들
        self.button_values = {
            "a": 0.0,
            "b": 0.0,
            "x": 0.0,
            "y": 0.0,
            "lb": 0.0,
            "rb": 0.0,
            "back": 0.0,
            "start": 0.0,
            "center": 0.0,
            "L": 0.0,
            "R": 0.0,
        }

        # 축 매핑 (키보드)
        self.axis_mapping = {
            "w": ("l_vertical", 1.0),
            "s": ("l_vertical", -1.0),
            "a": ("l_horizontal", 1.0),
            "d": ("l_horizontal", -1.0),
            "i": ("r_vertical", 1.0),
            "k": ("r_vertical", -1.0),
            "j": ("r_horizontal", 1.0),
            "l": ("r_horizontal", -1.0),
            "u": ("lt", 1.0),
            "o": ("rt", 1.0),
            # 방향키
        }

        # 버튼 매핑
        self.button_mapping = {
            "z": "a",
            "x": "b",
            "c": "x",
            "v": "y",
            "q": "lb",
            "e": "rb",
            "b": "back",
            "n": "start",
            "m": "center",
            "9": "L",
            "0": "R",
        }

        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_msg)
        self.timer.start(50)

        self.last_msg = Joy()

    def publish_msg(self):
        msg = Joy()

        for k, v in self.axis_values.items():
            setattr(msg, k, v)
        for k, v in self.button_values.items():
            setattr(msg, k, v)

        self.pub.publish(msg)
        self.last_msg = msg

    def handle_key_press(self, key):
        if key in self.axis_mapping:
            field, val = self.axis_mapping[key]
            self.axis_values[field] = val
            return True

        if key in self.button_mapping:
            btn = self.button_mapping[key]
            self.button_values[btn] = 1.0
            return True

        return False

    def handle_key_release(self, key):
        if key in self.axis_mapping:
            field, _ = self.axis_mapping[key]
            self.axis_values[field] = 0.0
            return True

        if key in self.button_mapping:
            btn = self.button_mapping[key]
            self.button_values[btn] = 0.0
            return True

        return False

    def get_current_values(self):
        merged = {}
        merged.update(self.axis_values)
        merged.update(self.button_values)
        return merged


# ==============================================================
# 3. GUI
# ==============================================================


class TeleopGUI(QWidget):
    def __init__(self, twist_pub, joy_pub):
        super().__init__()
        self.twist_pub = twist_pub
        self.joy_pub = joy_pub
        self.active_twist_keys = set()

        self.setWindowTitle("ROS Teleop (Twist + Joy)")
        self.initUI()
        self.setFocusPolicy(Qt.StrongFocus)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_values)
        self.timer.start(100)

    def initUI(self):
        vbox = QVBoxLayout()
        value_font = QFont("Courier New", 12)

        # Twist GUI
        twist_box = QGroupBox("Movement (Twist)")
        grid = QGridLayout()
        self.twist_labels = {}

        for i, (label, key) in enumerate(
            [
                ("Linear X (W/S)", "linear.x"),
                ("Linear Y (A/D)", "linear.y"),
                ("Angular Z (Q/E)", "angular.z"),
            ]
        ):
            grid.addWidget(QLabel(label + ":"), i, 0)
            val = QLabel("0.00")
            val.setFont(value_font)
            self.twist_labels[key] = val
            grid.addWidget(val, i, 1)

        twist_box.setLayout(grid)
        vbox.addWidget(twist_box)

        # Joy GUI
        joy_box = QGroupBox("Joy Message (/joy_msg)")
        grid2 = QGridLayout()
        self.joy_labels = {}

        for i, field in enumerate(JOY_FIELDS):
            grid2.addWidget(QLabel(field), i, 0)
            label = QLabel("0")
            label.setFont(value_font)
            label.setStyleSheet("color: blue;")
            self.joy_labels[field] = label
            grid2.addWidget(label, i, 1)

        joy_box.setLayout(grid2)
        vbox.addWidget(joy_box)

        self.setLayout(vbox)
        self.show()

    def update_values(self):
        # Twist GUI
        twist_vals = self.twist_pub.get_current_values()
        for k, v in twist_vals.items():
            self.twist_labels[k].setText(f"{v:.2f}")

        # Joy GUI
        joy_vals = self.joy_pub.get_current_values()
        for k, v in joy_vals.items():
            self.joy_labels[k].setText(f"{v:.2f}")
            if abs(v) > 0:
                self.joy_labels[k].setStyleSheet("color: red;")
            else:
                self.joy_labels[k].setStyleSheet("color: blue;")

    # 키 입력 처리
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.text().lower()
        kc = event.key()

        # Twist
        if key in self.twist_pub.key_bindings:
            self.twist_pub.publish_command(key)
            self.active_twist_keys.add(key)
            return

        if kc == Qt.Key_Space:
            self.twist_pub.stop()
            self.active_twist_keys.clear()
            return

        # Joy
        if self.joy_pub.handle_key_press(key):
            return

        # 방향키 (Joy)
        if kc == Qt.Key_Up:
            self.joy_pub.axis_values["dpad_vertical"] = 1.0
        if kc == Qt.Key_Down:
            self.joy_pub.axis_values["dpad_vertical"] = -1.0
        if kc == Qt.Key_Left:
            self.joy_pub.axis_values["dpad_horizontal"] = 1.0
        if kc == Qt.Key_Right:
            self.joy_pub.axis_values["dpad_horizontal"] = -1.0

    # 키 떼기 처리
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.text().lower()
        kc = event.key()

        if key in self.twist_pub.key_bindings:
            self.active_twist_keys.discard(key)
            if not self.active_twist_keys:
                self.twist_pub.stop()
            return

        if self.joy_pub.handle_key_release(key):
            return

        # 방향키 해제
        if kc in [Qt.Key_Up, Qt.Key_Down]:
            self.joy_pub.axis_values["dpad_vertical"] = 0.0
        if kc in [Qt.Key_Left, Qt.Key_Right]:
            self.joy_pub.axis_values["dpad_horizontal"] = 0.0


# ==============================================================
# MAIN
# ==============================================================

if __name__ == "__main__":
    try:
        rospy.init_node("keyboard_teleop_combined", anonymous=True)
    except rospy.exceptions.ROSException:
        pass

    app = QApplication(sys.argv)

    twist_pub = TwistPublisher()
    joy_pub = JoyPublisher()

    gui = TeleopGUI(twist_pub, joy_pub)

    sys.exit(app.exec_())
