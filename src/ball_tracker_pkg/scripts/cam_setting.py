#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ===== Qt/GL 소프트웨어 경로를 최우선으로 강제 (PyQt 임포트보다 먼저!) =====
import os
os.environ['QT_OPENGL'] = 'software'
os.environ['QT_XCB_GL_INTEGRATION'] = 'none'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
import sys, json, time, subprocess, re
import cv2
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QHBoxLayout, QVBoxLayout,
    QComboBox, QSlider, QCheckBox, QFileDialog, QMessageBox, QFormLayout,
    QGroupBox, QSpinBox
)
# --- ROS 라이브러리 임포트 ---
try:
    import rospy
    from rospy import core 
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS libraries not found. ROS Topic preview will be disabled.")
# ---------- OpenCV <-> 카메라 속성 매핑 ----------
CAP = cv2
PROP = {
    "BRIGHTNESS": CAP.CAP_PROP_BRIGHTNESS,
    "CONTRAST":   CAP.CAP_PROP_CONTRAST,
    "SATURATION": CAP.CAP_PROP_SATURATION,
    "HUE":        CAP.CAP_PROP_HUE,
    "GAIN":       CAP.CAP_PROP_GAIN,
    "EXPOSURE":   CAP.CAP_PROP_EXPOSURE, # exposure_absolute
    "WB_TEMPERATURE": CAP.CAP_PROP_WB_TEMPERATURE,
    "AUTO_EXPOSURE": getattr(CAP, "CAP_PROP_AUTO_EXPOSURE", 21), # exposure_auto
    "AUTO_WB":       getattr(CAP, "CAP_PROP_AUTO_WB", 45),       # white_balance_temperature_auto
}

# ⚠️ 파싱 결과가 없을 경우를 대비한 폴백(Fallback) 초기값 및 영역
FALLBACK_RANGES = {
    "BRIGHTNESS": (-64, 64), "CONTRAST": (0, 64), "SATURATION": (0, 128),
    "HUE": (-40, 40), "GAIN": (0, 100), "EXPOSURE": (1, 5000), # 최대값 5000으로 유지
    "WB_TEMPERATURE": (2800, 6500),
}
FALLBACK_DEFAULTS = {
    "BRIGHTNESS": 0, "CONTRAST": 32, "SATURATION": 64, "HUE": 0, "GAIN": 0, 
    "EXPOSURE": 157, "WB_TEMPERATURE": 4600, "AUTO_EXPOSURE": 1, "AUTO_WB": 1,
}

RES_LIST = [
    (640, 480), (800, 600), (1280, 720), (1280, 960),
    (1920, 1080), (2560, 1440), (3840, 2160)
]
FPS_LIST = [15, 30, 60, 120]

# --- 유틸리티 함수 (중략) ---

def bgr_to_qimage(frame):
    h, w = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888)
    
def _to_device_path(dev_text: str):
    s = dev_text.strip()
    if s.startswith('/dev/video'):
        return s
    try:
        idx = int(s)
        return f"/dev/video{idx}"
    except ValueError:
        return None

def _to_device_index(dev_text: str):
    s = dev_text.strip()
    try:
        return int(s)
    except ValueError:
        m = re.search(r'(\d+)$', s)
        return int(m.group(1)) if m else None

def _try_open_with_backends(dev, desired_fourccs=None):
    backends = [cv2.CAP_V4L2, getattr(cv2, 'CAP_V4L', cv2.CAP_ANY), cv2.CAP_ANY]
    desired_fourccs = desired_fourccs or ['YUYV', 'MJPG', 'H264']
    for be in backends:
        cap = cv2.VideoCapture(dev, be)
        if not cap.isOpened():
            if cap: cap.release()
            continue
        for fcc in desired_fourccs:
            fourcc = cv2.VideoWriter_fourcc(*fcc)
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            try: cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            except Exception: pass
            try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception: pass
            time.sleep(0.05)
            ok, _ = cap.read()
            if ok: return cap
        ok, _ = cap.read()
        if ok: return cap
        cap.release()
    return None

def _who_uses_device(dev_text: str):
    path = _to_device_path(dev_text)
    if not path: return ""
    try:
        out = subprocess.check_output(['fuser', '-v', path], stderr=subprocess.STDOUT, text=True)
        return out.strip()
    except Exception:
        return ""

def _parse_v4l2_ctrls(device_path: str) -> dict:
    """ v4l2-ctl 명령어를 실행하여 카메라 속성 값, 범위, 기본값을 파싱 """
    ctrls = {}
    if not device_path.startswith('/dev/video'):
        return ctrls
        
    try:
        cmd = ['v4l2-ctl', '-d', device_path, '--list-ctrls']
        result = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=1)
        output = result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError, TimeoutError) as e:
        print(f"Error running v4l2-ctl for {device_path}: {e}")
        return ctrls

    # OpenCV 속성 이름과 v4l2-ctl 출력 이름 매핑
    V4L2_NAME_MAP = {
        'brightness': "BRIGHTNESS", 'contrast': "CONTRAST", 'saturation': "SATURATION",
        'hue': "HUE", 'gain': "GAIN", 'exposure_absolute': "EXPOSURE",
        'white_balance_temperature': "WB_TEMPERATURE", 
        'exposure_auto': "AUTO_EXPOSURE", 'white_balance_temperature_auto': "AUTO_WB",
    }
    
    # 정규식 패턴: (1) 속성 이름, (2) 속성 타입, (3) 세부 정보 문자열, (4) 플래그 (flags=inactive 등)
    # flags는 선택적
    pattern = re.compile(r'\s*(\w+)\s+0x[0-9a-f]+\s+\((int|bool|menu)\)\s*:\s*(.*?)(?:\s+(flags=.+))?$')
    
    for line in output.splitlines():
        match = pattern.match(line)
        if match:
            name_v4l2, dtype, details, flags_str = match.groups()
            name_v4l2_lower = name_v4l2.lower()
            name_app = V4L2_NAME_MAP.get(name_v4l2_lower)
            if not name_app: continue

            prop_data = {'dtype': dtype, 'flags': flags_str if flags_str else ''}
            
            # 세부 정보 파싱 (min, max, default, value)
            for item in details.split():
                if '=' in item:
                    k, v = item.split('=', 1)
                    try: prop_data[k] = int(v) 
                    except ValueError: pass
            
            # Exposure Auto 및 WB Auto의 value 맵핑 보정 (OpenCV/GUI 기준)
            if name_app == "AUTO_EXPOSURE":
                # v4l2: 0=Manual, 1=Aperture Priority, 3=Continuous Auto
                # GUI/OpenCV: 0=Manual, 1=Auto/Continuous Auto
                # 우리는 0=Manual, 1/3=Auto로 간주하여 GUI 체크박스에 반영
                prop_data['value'] = 1 if prop_data.get('value') in [1, 3] else 0
                prop_data['default'] = 1 if prop_data.get('default') in [1, 3] else 0
            
            # 최종 저장
            ctrls[name_app] = {
                'min': prop_data.get('min'),
                'max': prop_data.get('max'),
                'default': prop_data.get('default'),
                'value': prop_data.get('value'),
                'is_active': 'inactive' not in prop_data['flags'] # flags=inactive 체크
            }
    return ctrls


class CamTuner(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Tuner (PyQt5 + OpenCV/ROS) - Preset Maker")
        self.cap = None
        self.ros_sub = None
        self.ros_frame = None # ROS 콜백에서 저장되는 CV Mat 이미지
        self.bridge = None
        self.playing = True
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.camera_ctrl_data = {} 
        
        # --- UI 구성 ---
        self.preview = QLabel("No camera / No ROS connection")
        self.preview.setFixedSize(960, 540)
        self.preview.setStyleSheet("background:#222; color:#aaa; border:1px solid #444;")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.mousePressEvent = self.toggle_play 
        self.devSel = QComboBox(); self.devSel.setEditable(True)
        for i in range(4): self.devSel.addItem(str(i))
        if ROS_AVAILABLE: 
            self.devSel.addItem("/camera/color/image_raw")
            self.devSel.addItem("/usb_cam/image_raw") # USB 캠 토픽 추가
        self.devSel.setCurrentIndex(0)
        self.resSel = QComboBox()
        for w, h in RES_LIST: self.resSel.addItem(f"{w}x{h}", (w, h))
        self.resSel.setCurrentText("1280x720"); self.resSel.setEnabled(True)
        self.fpsSel = QComboBox()
        for f in FPS_LIST: self.fpsSel.addItem(str(f))
        self.fpsSel.setCurrentText("30"); self.fpsSel.setEnabled(True)
        self.btnOpen = QPushButton("Open");  self.btnOpen.clicked.connect(self.open_camera)
        self.btnClose= QPushButton("Close"); self.btnClose.clicked.connect(self.close_camera)
        self.btnSave = QPushButton("Save Preset (JSON)"); self.btnSave.clicked.connect(self.save_preset)
        self.btnLoad = QPushButton("Load Preset"); self.btnLoad.clicked.connect(self.load_preset)
        self.btnReset = QPushButton("Reset to Default"); self.btnReset.clicked.connect(self.reset_to_default)
        top = QHBoxLayout()
        top.addWidget(QLabel("Source:")); top.addWidget(self.devSel); top.addWidget(QLabel("Res:"));    top.addWidget(self.resSel)
        top.addWidget(QLabel("FPS:"));    top.addWidget(self.fpsSel); top.addWidget(self.btnOpen); top.addWidget(self.btnClose)
        top.addStretch(1); top.addWidget(self.btnSave); top.addWidget(self.btnLoad); top.addWidget(self.btnReset) 
        
        self.sliders = {}; self.lblVals = {}; self.spinBoxes = {}; form = QFormLayout()
        
        # 슬라이더 및 SpinBox 생성 로직 (수정됨)
        def add_slider(name):
            rng = FALLBACK_RANGES.get(name, (0, 255)); default_val = FALLBACK_DEFAULTS.get(name, int(np.mean(rng)))
            
            # --- 위젯 생성 ---
            box = QHBoxLayout(); 
            sld = QSlider(Qt.Horizontal)
            spin = QSpinBox() # 값 직접 입력 위젯
            lbl = QLabel(str(default_val)) # 현재 값 레이블 (사용하지 않지만 구조 유지를 위해 남김)
            
            # --- 범위 설정 ---
            sld.setMinimum(rng[0]); sld.setMaximum(rng[1]); sld.setSingleStep(1); sld.setPageStep(1)
            spin.setRange(rng[0], rng[1])
            spin.setSingleStep(1)
            
            # --- 초기값 설정 ---
            sld.setValue(default_val); spin.setValue(default_val)
            
            # --- 연결 (Slider <-> SpinBox <-> Camera) ---
            # 1. Slider -> SpinBox
            # on_control_change는 SpinBox가 호출하도록 변경
            sld.valueChanged.connect(lambda v, s=spin: s.setValue(v))
            # 2. SpinBox -> Slider & Camera
            spin.valueChanged.connect(lambda v, d=sld, n=name: [d.setValue(v), self.on_control_change(n, v)])
            
            self.sliders[name] = sld; self.lblVals[name] = lbl 
            self.spinBoxes[name] = spin
            
            w = QWidget(); box.addWidget(sld); box.addWidget(spin);
            w.setLayout(box)
            form.addRow(QLabel(name), w)
            
        add_slider("BRIGHTNESS"); add_slider("CONTRAST"); add_slider("SATURATION"); add_slider("HUE")
        add_slider("GAIN"); add_slider("EXPOSURE"); add_slider("WB_TEMPERATURE")
        
        # 체크박스 기본값 설정
        self.cbAutoExp = QCheckBox("Auto Exposure"); self.cbAutoWB  = QCheckBox("Auto White Balance")
        self.cbAutoExp.setChecked(bool(FALLBACK_DEFAULTS["AUTO_EXPOSURE"])); self.cbAutoWB.setChecked(bool(FALLBACK_DEFAULTS["AUTO_WB"]))
        self.cbAutoExp.stateChanged.connect(self.on_auto_exp); self.cbAutoWB.stateChanged.connect(self.on_auto_wb)
        self.sliders["EXPOSURE"].setEnabled(not self.cbAutoExp.isChecked()); self.sliders["GAIN"].setEnabled(not self.cbAutoExp.isChecked())
        self.spinBoxes["EXPOSURE"].setEnabled(not self.cbAutoExp.isChecked()); self.spinBoxes["GAIN"].setEnabled(not self.cbAutoExp.isChecked())
        self.sliders["WB_TEMPERATURE"].setEnabled(not self.cbAutoWB.isChecked())
        self.spinBoxes["WB_TEMPERATURE"].setEnabled(not self.cbAutoWB.isChecked())

        autoBox = QVBoxLayout(); autoBox.addWidget(self.cbAutoExp); autoBox.addWidget(self.cbAutoWB)
        autoGroup = QGroupBox("Auto"); autoGroup.setLayout(autoBox)
        right = QVBoxLayout(); right.addLayout(form); right.addWidget(autoGroup); right.addStretch(1)
        rootL = QVBoxLayout(); rootL.addLayout(top); rootL.addWidget(self.preview, 1)
        root  = QHBoxLayout(); root.addLayout(rootL); root.addLayout(right)
        self.setLayout(root)
        
        self.open_camera()
        
    def ros_image_callback(self, msg):
        """ ROS Image 메시지를 받아서 OpenCV Mat으로 변환하고 self.ros_frame에 저장 """
        if self.bridge is None: return
        
        try:
            # ROS 이미지를 BGR 포맷으로 변환 (OpenCV 기본)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.ros_frame = cv_image
        except Exception as e:
            print(f"Error converting ROS image: {e}")
            
    def reset_to_default(self):
        """ v4l2-ctl 파싱 값의 'default' 항목으로 리셋 """
        
        # 1. Auto 설정 리셋
        for name in ["AUTO_EXPOSURE", "AUTO_WB"]:
            default_val = self.camera_ctrl_data.get(name, {}).get('default', FALLBACK_DEFAULTS.get(name))
            is_default_auto = bool(default_val)
            
            if name == "AUTO_EXPOSURE":
                self.cbAutoExp.setChecked(is_default_auto)
                self.on_auto_exp(Qt.Checked if is_default_auto else Qt.Unchecked)
            elif name == "AUTO_WB":
                self.cbAutoWB.setChecked(is_default_auto)
                self.on_auto_wb(Qt.Checked if is_default_auto else Qt.Unchecked)

        # 2. 수동 슬라이더 값 리셋 및 적용
        for name in ["BRIGHTNESS","CONTRAST","SATURATION","HUE","GAIN","EXPOSURE","WB_TEMPERATURE"]:
            default_val = self.camera_ctrl_data.get(name, {}).get('default', FALLBACK_DEFAULTS.get(name))
            
            if default_val is not None and name in self.sliders:
                v_min = self.sliders[name].minimum(); v_max = self.sliders[name].maximum()
                val_to_set = max(v_min, min(v_max, default_val))

                self.spinBoxes[name].setValue(val_to_set) # SpinBox가 Slider를 업데이트
                
        QMessageBox.information(self, "Reset", "Camera properties reset to hardware default values (via v4l2-ctl parsing).")
        
    def _get_alpha_beta(self):
        # ... (기존 로직 유지) ...
        max_contrast = self.sliders["CONTRAST"].maximum() if "CONTRAST" in self.sliders else 64
        contrast_factor = self.sliders["CONTRAST"].value() / (max_contrast / 2.0) if max_contrast > 0 else 1.0
        max_saturation = self.sliders["SATURATION"].maximum() if "SATURATION" in self.sliders else 128
        saturation_factor = self.sliders["SATURATION"].value() / (max_saturation / 2.0) if max_saturation > 0 else 1.0
        brightness = self.sliders["BRIGHTNESS"].value()
        alpha = contrast_factor; beta = brightness; return alpha, beta, saturation_factor
        
    def _update_ui_from_cap(self):
        """ OpenCV cap 객체에서 현재 Auto 상태만 읽어와 슬라이더 활성화/비활성화 반영 """
        if not self.cap or not self.cap.isOpened(): return
            
        try:
            auto_wb_val = self.cap.get(PROP["AUTO_WB"])
            is_auto_wb = auto_wb_val > 0.5
            self.cbAutoWB.setChecked(is_auto_wb)
            self.sliders["WB_TEMPERATURE"].setEnabled(not is_auto_wb)
            self.spinBoxes["WB_TEMPERATURE"].setEnabled(not is_auto_wb)
        except Exception: pass
            
        try:
            auto_exp_val = self.cap.get(PROP["AUTO_EXPOSURE"])
            is_auto_exp = auto_exp_val > 0.5
            self.cbAutoExp.setChecked(is_auto_exp) 
            self.sliders["EXPOSURE"].setEnabled(not is_auto_exp)
            self.sliders["GAIN"].setEnabled(not is_auto_exp)
            self.spinBoxes["EXPOSURE"].setEnabled(not is_auto_exp)
            self.spinBoxes["GAIN"].setEnabled(not is_auto_exp)
        except Exception: pass
            
    # ------------- 카메라 제어 -------------
    def open_camera(self):
        dev_text = self.devSel.currentText().strip()
        self.close_camera()
        
        # 1. ROS 토픽 구독 처리 (추가된 로직)
        if dev_text.startswith('/'): # ROS 토픽 이름으로 간주
            if not ROS_AVAILABLE:
                QMessageBox.critical(self, "ROS", "ROS libraries are not available. ROS Topic preview will be disabled."); return
            try:
                if not core.is_initialized(): # ROS 초기화 확인
                    QMessageBox.critical(self, "ROS", "ROS node is not initialized. Run the main function with ROS environment."); return

                self.bridge = CvBridge()
                # ROS Image 메시지 구독 시작
                self.ros_sub = rospy.Subscriber(dev_text, Image, self.ros_image_callback)
                self.timer.start(30) # 타이머 시작 (ROS 프레임을 표시하기 위해)
                self.playing = True
                self.preview.setText(f"Subscribing to: {dev_text}\n(Waiting for image...)")
                
                # ROS 모드에서는 카메라 속성 제어 비활성화
                self.resSel.setEnabled(False); self.fpsSel.setEnabled(False) 
                for name in self.sliders:
                    self.sliders[name].setEnabled(True) # 슬라이더 자체는 B/C/S 보정을 위해 활성화 유지
                    self.spinBoxes[name].setEnabled(True)
                
                QMessageBox.information(self, "ROS Topic", f"Subscribed to: {dev_text}")
                return # ROS 구독 후에는 OpenCV 로직을 건너뜀
            except Exception as e:
                QMessageBox.critical(self, "ROS Topic", f"Failed to subscribe to {dev_text}: {e}"); return


        # 2. OpenCV 카메라 열기
        dev_idx = _to_device_index(dev_text); dev_path = _to_device_path(dev_text); dev_for_cv = dev_idx if dev_idx is not None else dev_text
            
        # 2-1. v4l2-ctl 파싱을 통해 범위/현재값/기본값 읽기
        if dev_path:
            self.camera_ctrl_data = _parse_v4l2_ctrls(dev_path)
            
        # 2-2. 카메라 열기
        self.cap = _try_open_with_backends(dev_for_cv, desired_fourccs=['YUYV', 'MJPG', 'H264'])
            
        if not (self.cap and self.cap.isOpened()):
            QMessageBox.critical(self, "OpenCV", f"Failed to open camera: {dev_text}"); return
        
        w, h = self.resSel.currentData(); fps  = int(self.fpsSel.currentText())
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps); time.sleep(0.1)
        
        # 2-3. 파싱된 값으로 GUI와 카메라 설정 업데이트
        for name in ["BRIGHTNESS","CONTRAST","SATURATION","HUE","GAIN","EXPOSURE","WB_TEMPERATURE"]:
            ctrl_info = self.camera_ctrl_data.get(name, {})
            
            # 파싱된 범위, 현재값
            v_min = ctrl_info.get('min', FALLBACK_RANGES.get(name, (0, 255))[0])
            v_max = ctrl_info.get('max', FALLBACK_RANGES.get(name, (0, 255))[1])
            v_current = ctrl_info.get('value', FALLBACK_DEFAULTS.get(name))

            if name in self.sliders and v_min is not None and v_max is not None:
                slider = self.sliders[name]
                spin = self.spinBoxes[name]
                
                # 범위 설정 업데이트 (v4l2 파싱 결과를 우선 적용)
                slider.setMinimum(v_min); slider.setMaximum(v_max)
                spin.setRange(v_min, v_max)
                
                # 값 클리핑: min <= val_to_set <= max
                val_to_set = max(v_min, min(v_max, v_current)) if v_current is not None else slider.value()
                
                # GUI 값 업데이트 (SpinBox가 Slider를 업데이트하도록 연결)
                spin.setValue(val_to_set) 
                
                # 카메라 하드웨어에 값 적용 (클리핑된 값)
                self._set_prop(name, float(val_to_set))

        # 2-4. Auto 설정 업데이트 (체크박스 및 카메라)
        for name in ["AUTO_EXPOSURE", "AUTO_WB"]:
            ctrl_info = self.camera_ctrl_data.get(name, {})
            is_auto = bool(ctrl_info.get('value', FALLBACK_DEFAULTS.get(name))) 
            
            if name == "AUTO_EXPOSURE":
                self.cbAutoExp.setChecked(is_auto)
                self._set_prop(name, 0.75 if is_auto else 0.25)
            elif name == "AUTO_WB":
                self.cbAutoWB.setChecked(is_auto)
                self._set_prop(name, 1.0 if is_auto else 0.0)

        # 2-5. 최종 UI 상태 업데이트 및 실행
        self._update_ui_from_cap() 
        real_w  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)); real_h  = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.timer.start(30); self.playing = True; self.preview.setText("")
        self.resSel.setEnabled(True); self.fpsSel.setEnabled(True)
        QMessageBox.information(self, "Camera Opened", f"Device: {dev_text}\nActual: {real_w}x{real_h}\nLoaded properties via v4l2-ctl parsing.")
            
    def close_camera(self):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.ros_sub is not None:
            self.ros_sub.unregister()
            self.ros_sub = None
            self.ros_frame = None # 프레임 버퍼도 지움
            self.bridge = None # 브릿지도 지움
            # OpenCV 모드에서 ROS로 전환할 때 다시 활성화되도록 함
            for name in self.sliders:
                self.sliders[name].setEnabled(True)
                self.spinBoxes[name].setEnabled(True)

        self.preview.setText("No camera / No ROS connection")
        self.resSel.setEnabled(True); self.fpsSel.setEnabled(True)

    def on_timer(self):
        frame = None
        if self.cap:
            if self.playing:
                ok, f = self.cap.read()
                if ok: frame = f
        elif self.ros_sub:
            # ROS 구독 중일 때 self.ros_frame을 사용하여 표시
            if self.playing and self.ros_frame is not None:
                frame = self.ros_frame.copy()
                # 💡 ROS 토픽은 카메라 속성 제어가 안 되므로, B/C/S는 소프트웨어적으로 보정
                alpha, beta, saturation_factor = self._get_alpha_beta()
                frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                if saturation_factor != 1.0:
                    try:
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        h, s, v = cv2.split(hsv)
                        s = np.clip(s * saturation_factor, 0, 255).astype(np.uint8)
                        hsv = cv2.merge([h, s, v])
                        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                    except Exception:
                        pass
        
        if frame is not None:
            qimg = bgr_to_qimage(frame)
            self.preview.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.preview.width(), self.preview.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            ))
        elif self.ros_sub and self.playing:
             self.preview.setText(f"Subscribing to: {self.devSel.currentText().strip()}\n(Waiting for image...)")


    def toggle_play(self, _evt):
        if not self.cap and not self.ros_sub: return
        self.playing = not self.playing
        if not self.playing: self.preview.setText("Paused")

    def on_control_change(self, name, value):
        """ Slider 또는 SpinBox 값이 변경될 때 호출되며, 값 클리핑 후 카메라에 적용 """
        
        # ROS 모드에서는 B/C/S/HUE만 소프트웨어적으로 적용됨 (나머지는 무시)
        if self.ros_sub:
            if name in ["BRIGHTNESS", "CONTRAST", "SATURATION", "HUE"]:
                 # 슬라이더 값만 업데이트하고 실제 카메라 제어는 하지 않음
                 pass 
            return

        # OpenCV 모드 (기존 로직)
        if self.cap and self.cap.isOpened():
            # Auto 모드일 때 Exposure와 Gain은 변경 금지
            if name == "EXPOSURE" and self.cbAutoExp.isChecked(): return
            if name == "GAIN" and self.cbAutoExp.isChecked(): return
            if name == "WB_TEMPERATURE" and self.cbAutoWB.isChecked(): return
            
            # 💡 클리핑 로직: SpinBox에서 값이 넘어왔을 때 현재 Min/Max를 벗어나지 않도록 클리핑
            v_min = self.spinBoxes[name].minimum()
            v_max = self.spinBoxes[name].maximum()
            
            # v_min/v_max는 open_camera에서 v4l2 파싱 값으로 설정되지만, 한 번 더 확실하게 클리핑
            clipped_value = max(v_min, min(v_max, value))

            # 카메라 속성 설정
            self._set_prop(name, float(clipped_value))
            
            # Exposure, Gain, WB 변경 시에는 카메라 버퍼를 비워 갱신을 도움
            if name in ("EXPOSURE", "GAIN", "WB_TEMPERATURE"):
                for _ in range(4): self.cap.read()

    def on_auto_exp(self, state):
        is_auto = (state == Qt.Checked)
        
        # ROS 모드에서는 Exposure/Gain 제어는 비활성화됨
        if self.ros_sub:
             self.sliders["EXPOSURE"].setEnabled(False)
             self.sliders["GAIN"].setEnabled(False)
             self.spinBoxes["EXPOSURE"].setEnabled(False)
             self.spinBoxes["GAIN"].setEnabled(False)
             return

        if self.cap and self.cap.isOpened():
            # V4L2_AUTO_EXPOSURE_APERTURE_PRIORITY (0.75) 또는 V4L2_AUTO_EXPOSURE_SHUTTER_PRIORITY (0.25)
            self._set_prop("AUTO_EXPOSURE", 0.75 if is_auto else 0.25)
        self.sliders["EXPOSURE"].setEnabled(not is_auto)
        self.sliders["GAIN"].setEnabled(not is_auto)
        self.spinBoxes["EXPOSURE"].setEnabled(not is_auto)
        self.spinBoxes["GAIN"].setEnabled(not is_auto)

    def on_auto_wb(self, state):
        is_auto = (state == Qt.Checked)

        # ROS 모드에서는 WB 제어는 비활성화됨
        if self.ros_sub:
             self.sliders["WB_TEMPERATURE"].setEnabled(False)
             self.spinBoxes["WB_TEMPERATURE"].setEnabled(False)
             return
             
        if self.cap and self.cap.isOpened():
            self._set_prop("AUTO_WB", 1.0 if is_auto else 0.0)
        self.sliders["WB_TEMPERATURE"].setEnabled(not is_auto)
        self.spinBoxes["WB_TEMPERATURE"].setEnabled(not is_auto)

    def _set_prop(self, key, value):
        if not self.cap: return False
        prop_id = PROP.get(key)
        if prop_id is None: return False
        return self.cap.set(prop_id, value)

    def save_preset(self):

        home_dir = os.path.expanduser("~")
        default_path = os.path.join(home_dir, "soccer_ws", "camera_preset.json")

        path, _ = QFileDialog.getSaveFileName(self, "Save Preset", default_path, "JSON (*.json)")
        if not path: return
        data = {
            "BRIGHTNESS": self.sliders["BRIGHTNESS"].value(), "CONTRAST": self.sliders["CONTRAST"].value(),
            "SATURATION": self.sliders["SATURATION"].value(), "HUE": self.sliders["HUE"].value(),
            "GAIN": self.sliders["GAIN"].value(), "EXPOSURE": self.sliders["EXPOSURE"].value(),
            "WB_TEMPERATURE": self.sliders["WB_TEMPERATURE"].value() if "WB_TEMPERATURE" in self.sliders else None,
            "AUTO_EXPOSURE": int(self.cbAutoExp.isChecked()), "AUTO_WB": int(self.cbAutoWB.isChecked()),
            "RES": self.resSel.currentData() if self.resSel.isEnabled() else None,
            "FPS": int(self.fpsSel.currentText()) if self.fpsSel.isEnabled() else None,
            "DEVICE": self.devSel.currentText().strip(),
        }
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            QMessageBox.information(self, "Save Preset", f"Saved: {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Save Preset", f"Failed to save: {e}")

    def load_preset(self):
        
        home_dir = os.path.expanduser("~")
        default_path = os.path.join(home_dir, "soccer_ws", "camera_preset.json")
        
        path, _ = QFileDialog.getOpenFileName(self, "Load Preset", default_path, "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r") as f: data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Load Preset", f"Failed: {e}"); return

        is_auto_exp = bool(data.get("AUTO_EXPOSURE", 0)); is_auto_wb  = bool(data.get("AUTO_WB", 0))
        
        # Auto 체크박스 업데이트 (카메라 제어는 SpinBox 업데이트 후 on_control_change에서 처리됨)
        self.cbAutoExp.setChecked(is_auto_exp); self.cbAutoWB.setChecked(is_auto_wb)

        dev = data.get("DEVICE", None)
        if dev is not None:
            if self.devSel.findText(str(dev)) < 0: self.devSel.addItem(str(dev))
            self.devSel.setCurrentText(str(dev))

        res = tuple(data.get("RES", (1280, 720))); idx = self.resSel.findText(f"{res[0]}x{res[1]}")
        if idx >= 0: self.resSel.setCurrentIndex(idx)
        fps = int(data.get("FPS", 30)); idx = self.fpsSel.findText(str(fps))
        if idx >= 0: self.fpsSel.setCurrentIndex(idx)

        for k in ["BRIGHTNESS","CONTRAST","SATURATION","HUE","GAIN","EXPOSURE","WB_TEMPERATURE"]:
            if k in data and self.sliders.get(k):
                val_from_file = int(data[k])
                
                # 💡 파일 로드 시 클리핑 로직 강화 (min/max는 현재 SpinBox 범위 기준)
                v_min = self.spinBoxes[k].minimum(); v_max = self.spinBoxes[k].maximum()
                val_to_set = max(v_min, min(v_max, val_from_file))
                
                # SpinBox를 통해 슬라이더와 카메라 모두 업데이트
                self.spinBoxes[k].setValue(val_to_set)

        QMessageBox.information(self, "Load Preset", f"Loaded: {os.path.basename(path)}")
        
def main():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL)
    
    if ROS_AVAILABLE:
        try:
            if not core.is_initialized():
                 # ROS 환경에서 실행 시 기존 노드와 충돌 방지 및 신호 제어 비활성화
                 rospy.init_node('camera_tuner_gui', anonymous=True, disable_signals=True)
                 print("ROS node initialized for topic subscription.")
            else:
                 print("ROS node already initialized.")
        except rospy.exceptions.ROSInitException as e:
            print(f"Warning: ROS initialization failed. ROS topic feature disabled. {e}")
            
    app = QApplication(sys.argv)
    w = CamTuner()
    w.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()