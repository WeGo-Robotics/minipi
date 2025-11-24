#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ===== RK3588 등에서 OpenGL 가속 문제 회피(소프트웨어 렌더링 강제) =====
import os
os.environ['QT_OPENGL'] = 'software'
os.environ['QT_XCB_GL_INTEGRATION'] = 'none'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'

import sys, time, traceback
from collections import deque

# -------------------- ROS 모듈 로드 상태 점검 --------------------
HAVE_ROSPY = False
HAVE_CVBRIDGE = False
HAVE_OPENCV = False
try:
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, CompressedImage
    HAVE_ROSPY = True
except Exception as e:
    print("[ROS 체크] rospy 임포트 실패:", repr(e))
    traceback.print_exc()

try:
    import cv2
    import numpy as np
    HAVE_OPENCV = True
except Exception as e:
    print("[ROS 체크] OpenCV/NumPy 임포트 실패:", repr(e))
    traceback.print_exc()

if HAVE_ROSPY:
    try:
        from cv_bridge import CvBridge, CvBridgeError
        HAVE_CVBRIDGE = True
    except Exception as e:
        print("[ROS 체크] cv_bridge 임포트 실패:", repr(e))
        traceback.print_exc()

# roscore 연결 확인
def is_master_available():
    if not HAVE_ROSPY:
        return False
    try:
        import rosgraph
        m = rosgraph.Master('/yolo_gui_topic_picker_probe')
        m.getPid()
        return True
    except Exception as e:
        print("[ROS 체크] roscore 연결 실패:", repr(e))
        return False

# -------------------- Qt --------------------
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui  import QImage, QPixmap, QColor, QTextCharFormat, QTextCursor, QTextBlockFormat, QFont
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLineEdit, QPushButton, QFormLayout, QMessageBox, QComboBox,
    QTextEdit 
)

# ---- QImage Format 호환 (PyQt5/6) ----
try:
    FMT_RGB888 = QImage.Format_RGB888           # PyQt5
except AttributeError:
    FMT_RGB888 = QImage.Format.Format_RGB888    # PyQt6

APP_ORG  = "wego"
APP_NAME = "yolo_gui_topic_picker"

def rosparam(name, default):
    if not HAVE_ROSPY:
        return default
    try:
        return rospy.get_param(name, default)
    except Exception:
        return default

# -------------------- 데이터 저장소 --------------------
class DetectionStore:
    def __init__(self, max_age=0.8):
        self.boxes = []
        self.stamp = 0.0
        self.max_age = max_age
    def update(self, boxes):
        self.boxes = boxes
        self.stamp = time.time()
    def get(self):
        if time.time() - self.stamp > self.max_age:
            return []
        return list(self.boxes)

class ImageStore:
    def __init__(self):
        self.frame = None
        self.last_ts = time.time()
        self.fps_hist = deque(maxlen=30)
    def update(self, frame):
        now = time.time()
        self.frame = frame
        dt = max(1e-6, now - self.last_ts)
        self.last_ts = now
        self.fps_hist.append(1.0 / dt)
    def get(self):
        return None if self.frame is None else self.frame.copy()
    def fps(self):
        return 0.0 if not self.fps_hist else sum(self.fps_hist)/len(self.fps_hist)

# -------------------- 메인 GUI --------------------
class YoloGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YOLO GUI Viewer (ROS1 + Topic Picker)")
        self.resize(1100, 720)

        # 파라미터
        self.default_img_topic = rosparam("~image_topic", "/camera/color/image_raw")
        self.default_det_topic = rosparam("~detection_topic", "/yolo/detections")
        self.draw_thickness    = int(rosparam("~thickness", 2))
        self.font_scale        = float(rosparam("~font_scale", 0.6))
        self.conf_threshold    = float(rosparam("~conf_threshold", 0.0))

        # 상태
        self.bridge   = CvBridge() if (HAVE_ROSPY and HAVE_CVBRIDGE) else None
        self.img_sub  = None
        self.det_sub  = None
        self.img_store = ImageStore()
        self.det_store = DetectionStore(max_age=0.8)

        # LLM 로딩 상태 추적 변수 **추가**
        self.is_waiting_for_llm = False
        self.loading_spinner_text = "..." # 로딩 인디케이터 텍스트
        
        # 상단 컨트롤 (토픽 선택)
        top = QHBoxLayout()
        top.addWidget(QLabel("영상 토픽"))
        self.imgTopicCombo = QComboBox()
        self.refreshBtn = QPushButton("토픽 새로고침")
        self.subscribeBtn = QPushButton("구독 시작")
        self.subscribeBtn.setCheckable(True)
        top.addWidget(self.imgTopicCombo, 1)
        top.addWidget(self.refreshBtn)
        top.addWidget(self.subscribeBtn)

        # 영상 표시
        self.label_img = QLabel("영상 미표시")
        self.label_img.setAlignment(Qt.AlignCenter)
        self.label_img.setMinimumHeight(240) 
        # 레이아웃이 수평 분할되므로, 영상은 left_container의 남은 공간을 모두 사용합니다.
        self.label_img.setStyleSheet("background:#0b1220; border:1px solid #27324a; border-radius:8px;")

        self.info_lbl = QLabel("")
        self.info_lbl.setStyleSheet("color:#9aa7bd;")

        # 채팅 기록 창
        self.chat_log = QTextEdit()
        self.chat_log.setReadOnly(True)
        self.chat_log.setMinimumHeight(150)
        
        # 배경색 흰색, 폰트 크기 설정
        self.chat_log.setStyleSheet(
            "background: white; "
            "border:1px solid #444444; "
            "border-radius:8px; "
            "color: #333333; "
            "font-size: 30pt; " # 폰트 크기 크게
        )

        # 입력
        input_box = QGroupBox("User Input (LLM Command)")
        self.input_edit = QLineEdit()
        self.input_edit.setPlaceholderText("메시지를 입력하고 Send를 누르면 /gui/input 으로 발행됩니다.")
        self.btn_send = QPushButton("Send")
        r = QHBoxLayout()
        r.addWidget(self.input_edit, 1)
        r.addWidget(self.btn_send)
        fl = QFormLayout(); fl.addRow(r)
        input_box.setLayout(fl)

        # =======================================================
        # 메인 레이아웃: 영상(왼쪽), 대화(오른쪽)
        # =======================================================
        
        # 1. 왼쪽 컨테이너 (영상 및 컨트롤) - QVBoxLayout
        left_container = QVBoxLayout()
        left_container.addLayout(top)
        left_container.addWidget(self.label_img, 1) 
        left_container.addWidget(self.info_lbl)
        
        # 2. 오른쪽 컨테이너 (대화 창 및 입력) - QVBoxLayout
        right_container = QVBoxLayout()
        right_container.addWidget(self.chat_log, 1) 
        right_container.addWidget(input_box)
        
        # 3. 메인 레이아웃 (수평 분할) - QHBoxLayout
        main_lay = QHBoxLayout(self)
        main_lay.addLayout(left_container, 2) # 영상 영역
        main_lay.addLayout(right_container, 1) # 대화 영역
        
        # =======================================================

        # 연결
        self.refreshBtn.clicked.connect(self.fill_image_topics)
        self.subscribeBtn.toggled.connect(self.toggle_subscribe)
        self.btn_send.clicked.connect(self.on_send)

        # 퍼블리셔
        self.user_pub = rospy.Publisher("/gui/input", String, queue_size=5) if HAVE_ROSPY else None

        # LLM 응답 구독
        self.rkllama_sub=rospy.Subscriber("/rkllama/output", String, self.rkllama_output_cb)

        # ROS 초기화 & 상태
        self._video_enabled = False
        self._init_ros_and_ui()

        # YOLO 검출 구독
        if self._video_enabled:
            self.det_sub = rospy.Subscriber(self.default_det_topic, rospy.AnyMsg, self.cb_detection_any, queue_size=2)

        # 타이머
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(30)

    # ---------- LLM 응답 콜백 (채팅 기록 업데이트) ----------
    def rkllama_output_cb(self, msg: String):
        # 1. 로딩 인디케이터 제거 **수정**
        self._remove_loading_indicator()
        
        answer = msg.data
        # LLM(로봇)은 왼쪽에 표시
        # LLM 응답은 파란색 계열, 왼쪽 정렬
        self._append_to_chat_log("LLM", answer, QColor("#1e90ff"), Qt.AlignLeft) # 짙은 파랑

    # ---------- 채팅 기록에 메시지 추가 헬퍼 함수 (로딩 플래그 추가) ----------
    def _append_to_chat_log(self, sender, text, color, alignment, is_loading=False): 
        cursor = self.chat_log.textCursor()
        cursor.movePosition(QTextCursor.End)
        
        # 1. 블록 포맷 설정 (정렬)
        block_format = QTextBlockFormat()
        block_format.setAlignment(alignment)
        cursor.insertBlock(block_format)

        # 2. 메시지 포맷 설정 (색상, **굵기**)
        format_text = QTextCharFormat()
        format_text.setForeground(color)
        
        # 굵은 폰트 설정
        font = QFont()
        font.setPointSize(24) # 폰트 크기 유지 (QTextEdit 스타일시트와 동일하게)
        if not is_loading:
            font.setBold(True) # 로딩 메시지는 굵게 하지 않음
        format_text.setFont(font)
        
        # 3. 메시지 텍스트 준비
        message_text = "[" + sender + "] " + text.replace('\n', ' ') 
        
        # 4. 텍스트 삽입
        cursor.insertText(message_text, format_text)
        
        self.chat_log.setTextCursor(cursor)
        self.chat_log.ensureCursorVisible()

    # ---------- 로딩 인디케이터 헬퍼 함수 **추가** ----------
    def _append_loading_indicator(self):
        """LLM 응답을 기다리는 동안 임시 로딩 메시지를 표시합니다."""
        if not self.is_waiting_for_llm:
            self.is_waiting_for_llm = True
            # LLM 위치(왼쪽)에 표시, 회색 계열로 표시
            # is_loading=True를 전달하여 굵은 글씨를 방지
            self._append_to_chat_log("LLM", self.loading_spinner_text, QColor("#888888"), Qt.AlignLeft, is_loading=True)

    def _remove_loading_indicator(self):
        """로딩 메시지를 제거하고 실제 LLM 응답을 위한 공간을 확보합니다."""
        if not self.is_waiting_for_llm:
            return
            
        cursor = self.chat_log.textCursor()
        cursor.movePosition(QTextCursor.End)
        
        # 현재 커서가 있는 줄(블록)을 선택
        cursor.select(QTextCursor.BlockUnderCursor)
        last_block_text = cursor.selectedText().strip()

        # 마지막 블록이 로딩 표시와 일치하면 제거
        expected_text = f"[LLM] {self.loading_spinner_text}"
        if last_block_text == expected_text:
            cursor.removeSelectedText()
            cursor.deletePreviousChar() # 블록 경계를 정리 (옵션)
        
        self.is_waiting_for_llm = False

    # ---------- ROS 초기화 & UI 활성화 판단 ----------
    def _init_ros_and_ui(self):
        status_msgs = []
        if not HAVE_ROSPY:
            status_msgs.append("rospy 미탑재")
        if not HAVE_OPENCV:
            status_msgs.append("OpenCV/NumPy 미탑재")
        if not HAVE_CVBRIDGE:
            status_msgs.append("cv_bridge 없음(CompressedImage만 지원)")

        master_ok = is_master_available()
        if HAVE_ROSPY and not rospy.core.is_initialized():
            try:
                rospy.init_node("yolo_gui_topic_picker", anonymous=True, disable_signals=True)
            except Exception as e:
                status_msgs.append(f"ROS 노드 init 실패: {e}")

        master_ok = master_ok or is_master_available()
        if HAVE_ROSPY and master_ok and HAVE_OPENCV:
            self._video_enabled = True
            self.imgTopicCombo.setEnabled(True)
            self.refreshBtn.setEnabled(True)
            self.subscribeBtn.setEnabled(True)
            self.fill_image_topics()
            self._select_default_image_topic()
            base = "ROS 활성"
            if not HAVE_CVBRIDGE:
                base += " · cv_bridge 없음 → Image 토픽은 비권장(Compressed 권장)"
            self.info_lbl.setText(base)
        else:
            self._video_enabled = False
            self.imgTopicCombo.setEnabled(False)
            self.refreshBtn.setEnabled(False)
            self.subscribeBtn.setEnabled(False)
            reason = "ROS 비활성: " + (", ".join(status_msgs) if status_msgs else "환경 미구성")
            self.info_lbl.setText(reason)
            print("[상태] " + reason)

    # ---------- 토픽 목록 ----------
    def fill_image_topics(self):
        self.imgTopicCombo.clear()
        if not self._video_enabled:
            self.imgTopicCombo.addItem("(영상 UI 비활성)")
            return
        try:
            topics = rospy.get_published_topics()
        except Exception:
            self.imgTopicCombo.addItem("(토픽 조회 실패)")
            return

        entries = []
        for name, typ in topics:
            if typ == "sensor_msgs/CompressedImage":
                entries.append((name, typ, True, True))
            elif typ == "sensor_msgs/Image":
                entries.append((name, typ, False, HAVE_CVBRIDGE))
        entries.sort(key=lambda x: x[0].lower())

        if not entries:
            self.imgTopicCombo.addItem("(영상 토픽 없음)")
            return

        for name, typ, is_comp, usable in entries:
            label = f"{name}  [{typ}]"
            if not usable:
                label += "  (cv_bridge 필요)"
            self.imgTopicCombo.addItem(label, (name, is_comp, usable))

    def _select_default_image_topic(self):
        for i in range(self.imgTopicCombo.count()):
            data = self.imgTopicCombo.itemData(i)
            if not data:
                continue
            name, is_comp, usable = data
            if name == self.default_img_topic or name.rstrip("/compressed") == self.default_img_topic:
                self.imgTopicCombo.setCurrentIndex(i)
                return
    
    # ---------- 구독 ----------
    def toggle_subscribe(self, checked):
        if not self._video_enabled:
            self.subscribeBtn.setChecked(False)
            return
        if checked:
            data = self.imgTopicCombo.currentData()
            if not data:
                QMessageBox.warning(self, "경고", "구독할 영상 토픽이 없습니다.")
                self.subscribeBtn.setChecked(False); return
            name, is_comp, usable = data
            if not usable:
                QMessageBox.warning(self, "경고", "cv_bridge가 없어 해당 Image 토픽을 처리할 수 없습니다. CompressedImage를 선택하세요.")
                self.subscribeBtn.setChecked(False); return
            self.subscribe_image(name, is_comp)
            self.subscribeBtn.setText("구독 중지")
        else:
            self.unsubscribe_image()
            self.subscribeBtn.setText("구독 시작")
            self.info_lbl.setText("정지")

    def subscribe_image(self, topic, is_compressed):
        self.unsubscribe_image()
        if is_compressed:
            self.img_sub = rospy.Subscriber(topic, CompressedImage, self.cb_image_compressed, queue_size=1)
        else:
            self.img_sub = rospy.Subscriber(topic, Image, self.cb_image, queue_size=1)
        self.info_lbl.setText(f"구독: {topic}")

    def unsubscribe_image(self):
        if self.img_sub is not None:
            try: self.img_sub.unregister()
            except Exception: pass
            self.img_sub = None

    # ---------- ROS 콜백: 이미지 ----------
    def cb_image(self, msg):
        if not HAVE_CVBRIDGE:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        self.img_store.update(frame)

    def cb_image_compressed(self, msg):
        if not HAVE_OPENCV:
            return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception:
            return
        self.img_store.update(frame)

    # ---------- ROS 콜백: 검출(자동 감지) ----------
    def cb_detection_any(self, any_msg):
        mtype = any_msg._connection_header.get('type', '')
        boxes = []
        try:
            if "vision_msgs/Detection2DArray" in mtype:
                boxes = self._parse_vision_msgs(any_msg)
            elif "darknet_ros_msgs/BoundingBoxes" in mtype:
                boxes = self._parse_darknet_ros(any_msg)
            else:
                boxes = self._parse_custom(any_msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[det-parse] {mtype}: {e}")
            boxes = []
        if self.conf_threshold > 0:
            boxes = [b for b in boxes if (b[5] is None or b[5] >= self.conf_threshold)]
        self.det_store.update(boxes)

    def _parse_vision_msgs(self, msg):
        from vision_msgs.msg import Detection2DArray
        arr = Detection2DArray(); arr.deserialize(msg._buff)
        out=[]
        for det in arr.detections:
            cx,cy = det.bbox.center.x, det.bbox.center.y
            w,h   = det.bbox.size_x, det.bbox.size_y
            x1,y1 = int(cx-w/2.0), int(cy-h/2.0)
            x2,y2 = int(cx+w/2.0), int(cy+h/2.0)
            label=""; conf=None
            if det.results:
                label = det.results[0].hypothesis.class_id
                conf  = float(det.results[0].hypothesis.score)
            out.append((x1,y1,x2,y2,label,conf))
        return out

    def _parse_darknet_ros(self, msg):
        from darknet_ros_msgs.msg import BoundingBoxes
        bb = BoundingBoxes(); bb.deserialize(msg._buff)
        out=[]
        for b in bb.bounding_boxes:
            out.append((int(b.xmin), int(b.ymin), int(b.xmax), int(b.ymax), b.Class, float(b.probability)))
        return out

    def _parse_custom(self, msg):
        out=[]
        try:
            has_arrays = all(hasattr(msg, f) for f in ["x_center","y_center","width","height"])
            if has_arrays:
                xs=list(getattr(msg,"x_center")); ys=list(getattr(msg,"y_center"))
                ws=list(getattr(msg,"width"));    hs=list(getattr(msg,"height"))
                labels = list(getattr(msg,"label")) if hasattr(msg,"label") else \
                         list(getattr(msg,"class_name")) if hasattr(msg,"class_name") else [""]*len(xs)
                confs  = list(getattr(msg,"confidence")) if hasattr(msg,"confidence") else \
                         list(getattr(msg,"conf")) if hasattr(msg,"conf") else [None]*len(xs)
                n=min(len(xs),len(ys),len(ws),len(hs),len(labels),len(confs))
                for i in range(n):
                    cx,cy,w,h=xs[i],ys[i],ws[i],hs[i]
                    x1,y1 = int(cx-w/2.0), int(cy-h/2.0)
                    x2,y2 = int(cx+w/2.0), int(cy+h/2.0)
                    out.append((x1,y1,x2,y2,str(labels[i]), None if confs[i] is None else float(confs[i])))
                if out: return out
        except Exception:
            pass
        try:
            if hasattr(msg,"boxes"):
                for b in msg.boxes:
                    if all(hasattr(b,k) for k in ["xmin","ymin","xmax","ymax"]):
                        x1,y1,x2,y2 = int(b.xmin),int(b.ymin),int(b.xmax),int(b.ymax)
                    elif all(hasattr(b,k) for k in ["x","y","w","h"]):
                        x1,y1 = int(b.x-b.w/2.0), int(b.y-b.h/2.0)
                        x2,y2 = int(b.x+b.w/2.0), int(b.y+b.h/2.0)
                    else:
                        continue
                    label = getattr(b,"label","")
                    conf  = getattr(b,"conf",None)
                    conf  = None if conf is None else float(conf)
                    out.append((x1,y1,x2,y2,str(label),conf))
        except Exception:
            pass
        return out

    # ---------- 주기 갱신 ----------
    def on_timer(self):
        frame = self.img_store.get()
        if frame is None:
            return
        overlay = frame.copy()
        for (x1,y1,x2,y2,label,conf) in self.det_store.get():
            cv2.rectangle(overlay,(x1,y1),(x2,y2),(0,255,0),self.draw_thickness)
            tag = label if label else "obj"
            if conf is not None: tag = f"{tag} {conf:.2f}"
            (tw,th), bl = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, 1)
            y_text = max(0, y1-4)
            cv2.rectangle(overlay,(x1,y_text-th-bl),(x1+tw+4,y_text),(0,255,0),-1)
            cv2.putText(overlay, tag, (x1+2,y_text-2), cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0,0,0),1,cv2.LINE_AA)

        # BGR -> RGB, QImage 변환 (호환 포맷 사용)
        rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        # self.label_img.size() 대신 self.label_img.width(), height() 사용
        w_target = self.label_img.width()
        h_target = self.label_img.height()
        qimg = QImage(rgb.data, w, h, ch * w, FMT_RGB888)
        pix  = QPixmap.fromImage(qimg).scaled(w_target, h_target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label_img.setPixmap(pix)

        fps = self.img_store.fps()
        info = f"Image: {self.current_image_topic()} | Det: {self.default_det_topic} | FPS: {fps:4.1f} | Boxes: {len(self.det_store.get())}"
        self.info_lbl.setText(info)

    def current_image_topic(self):
        data = self.imgTopicCombo.currentData()
        return data[0] if data else "-"

    # ---------- 입력 발행 (채팅 기록 업데이트 포함) ----------
    @pyqtSlot()
    def on_send(self):
        text = self.input_edit.text().strip()
        if not text:
            QMessageBox.information(self, "Info", "보낼 텍스트를 입력하세요.")
            return
            
        # 1. 채팅 기록에 사용자 메시지 추가 (오른쪽 정렬)
        # User 응답은 초록색 계열, 오른쪽 정렬
        self._append_to_chat_log("User", text, QColor("#008000"), Qt.AlignRight) # 짙은 초록
        
        # 2. 로딩 인디케이터 추가 **추가**
        self._append_loading_indicator()

        # 3. ROS 토픽으로 발행
        if self.user_pub:
            self.user_pub.publish(String(data=text))
            
        # 4. 입력창 비우기
        self.input_edit.clear()

# -------------------- main --------------------
def main():
    if HAVE_ROSPY:
        print("ROS_MASTER_URI =", os.environ.get("ROS_MASTER_URI"))
        print("ROS_PACKAGE_PATH =", os.environ.get("ROS_PACKAGE_PATH"))
    else:
        print("[경고] rospy 미탑재 — ROS setup.bash를 source 하세요.")

    if HAVE_ROSPY and not is_master_available():
        print("[안내] roscore 연결이 안 보입니다. `roscore`가 켜져 있거나 ROS_MASTER_URI가 올바른지 확인하세요.")

    app = QApplication(sys.argv)
    app.setOrganizationName(APP_ORG)
    app.setApplicationName(APP_NAME)
    w = YoloGui()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()