#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ===== RK3588 등에서 OpenGL 가속 문제 회피(소프트웨어 렌더링 강제) =====
import os
os.environ['QT_OPENGL'] = 'software'
os.environ['QT_XCB_GL_INTEGRATION'] = 'none'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'

import sys, glob, time
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

# --- ROS 의존 (선택적): 없으면 영상 기능 비활성화 ---
ROS_OK = True
try:
    import rospy
    from sensor_msgs.msg import Image as RosImage
    from sensor_msgs.msg import CompressedImage as RosCompressedImage
    import cv2
    from cv_bridge import CvBridge, CvBridgeError
    import numpy as np
except Exception:
    ROS_OK = False

# rospkg는 ROS1에 포함. 없으면: sudo apt install -y python3-rospkg
try:
    import rospkg
except ImportError:
    print("rospkg 모듈이 필요합니다. `sudo apt install python3-rospkg` 후 재시도.")
    sys.exit(1)

APP_ORG  = "wego"
APP_NAME = "ros-launch-manager-pretty"


# ============================== roslaunch 프로세스 ==============================
class LaunchProcess(QtCore.QObject):
    textAppended = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, package: str, rel_launch_path: str, env: dict = None):
        super().__init__()
        self.package = package
        self.rel_launch_path = rel_launch_path
        self.qproc = QtCore.QProcess()
        if env:
            qenv = QtCore.QProcessEnvironment.systemEnvironment()
            for k, v in env.items():
                qenv.insert(k, v)
            self.qproc.setProcessEnvironment(qenv)
        self.qproc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.qproc.readyRead.connect(self._on_ready_read)
        self.qproc.finished.connect(self._on_finished)

    def start(self):
        self.qproc.start("roslaunch", [self.package, self.rel_launch_path])

    def _on_ready_read(self):
        data = bytes(self.qproc.readAll()).decode("utf-8", errors="replace")
        self.textAppended.emit(data)

    def _on_finished(self, *_):
        self.textAppended.emit("\n[roslaunch exited]\n")
        self.finished.emit()

    def is_running(self):
        return self.qproc.state() == QtCore.QProcess.Running

    def pid(self):
        return int(self.qproc.processId()) if self.qproc.processId() != 0 else None

    def terminate_gracefully(self):
        if self.is_running():
            self.qproc.terminate()
            QtCore.QTimer.singleShot(2500, self.kill_force)

    def kill_force(self):
        if self.is_running():
            self.qproc.kill()


# ============================== 런치 파일 스캔 ==============================
def find_launch_files(limit_packages=None):
    rp = rospkg.RosPack()
    packages = rp.list()
    if limit_packages:
        packages = [p for p in packages if p in limit_packages]

    result = {}
    for pkg in packages:
        try:
            pkg_path = Path(rp.get_path(pkg))
        except rospkg.ResourceNotFound:
            continue
        launch_dir = pkg_path / "launch"
        if not launch_dir.is_dir():
            continue
        files = [Path(p) for p in glob.glob(str(launch_dir / "**" / "*.launch"), recursive=True)]
        if not files:
            continue
        entries = []
        for f in files:
            rel = f.relative_to(launch_dir)
            entries.append((str(rel).replace(os.sep, "/"), str(f)))
        result[pkg] = sorted(entries, key=lambda x: x[0].lower())
    return result


# ============================== 영상 구독기 (선택적) ==============================
class RosImageSubscriber(QtCore.QObject):
    """ROS 이미지 토픽을 구독해 numpy frame을 내보냄."""
    frameReady = QtCore.pyqtSignal(object, int, int, float)  # frame, w, h, fps

    def __init__(self, parent=None):
        super().__init__(parent)
        self.bridge = CvBridge() if ROS_OK else None
        self.sub = None
        self.last_ts = None
        self._topic = None
        self._is_compressed = False

    def subscribe(self, topic: str, is_compressed: bool):
        self.unsubscribe()
        if not ROS_OK:
            return
        self._topic = topic
        self._is_compressed = is_compressed
        if is_compressed:
            self.sub = rospy.Subscriber(topic, RosCompressedImage, self._cb_compressed, queue_size=1)
        else:
            self.sub = rospy.Subscriber(topic, RosImage, self._cb_image, queue_size=1)

    def unsubscribe(self):
        if self.sub is not None:
            self.sub.unregister()
            self.sub = None

    def _emit_frame(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        now = time.time()
        fps = 0.0
        if self.last_ts is not None:
            dt = now - self.last_ts
            if dt > 0:
                fps = 1.0 / dt
        self.last_ts = now
        self.frameReady.emit(frame_bgr, w, h, fps)

    def _cb_image(self, msg: 'RosImage'):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return
        self._emit_frame(frame)

    def _cb_compressed(self, msg: 'RosCompressedImage'):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception:
            return
        self._emit_frame(frame)


# ============================== 카드형 위젯들 ==============================
class LaunchItemWidget(QtWidgets.QWidget):
    startClicked = QtCore.pyqtSignal()
    stopClicked  = QtCore.pyqtSignal()

    def __init__(self, pkg: str, rel_path: str):
        super().__init__()
        self.pkg = pkg
        self.rel_path = rel_path

        title = QtWidgets.QLabel(rel_path)
        title.setObjectName("itemTitle")

        self.badge = QtWidgets.QLabel("stopped")
        self.badge.setObjectName("badgeStopped")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setMinimumWidth(72)

        self.pid_lbl = QtWidgets.QLabel("")
        self.pid_lbl.setObjectName("pidLabel")
        self.pid_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.pid_lbl.setMinimumWidth(60)

        self.btn_start = QtWidgets.QToolButton()
        self.btn_start.setText("실행")
        self.btn_start.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.btn_start.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.btn_start.clicked.connect(lambda *_,: self.startClicked.emit())

        self.btn_stop = QtWidgets.QToolButton()
        self.btn_stop.setText("정지")
        self.btn_stop.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.btn_stop.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(lambda *_,: self.stopClicked.emit())

        row = QtWidgets.QHBoxLayout()
        row.addWidget(title, 1)
        row.addWidget(self.badge)
        row.addWidget(self.pid_lbl)
        row.addWidget(self.btn_start)
        row.addWidget(self.btn_stop)

        card = QtWidgets.QFrame()
        card.setObjectName("card")
        card.setLayout(row)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(card)

    def setRunning(self, running: bool, pid_text: str = ""):
        if running:
            self.badge.setText("running")
            self.badge.setObjectName("badgeRunning")
            self._refresh(self.badge)
            self.pid_lbl.setText(pid_text)
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(True)
        else:
            self.badge.setText("stopped")
            self.badge.setObjectName("badgeStopped")
            self._refresh(self.badge)
            self.pid_lbl.setText("")
            self.btn_start.setEnabled(True)
            self.btn_stop.setEnabled(False)

    def _refresh(self, w):
        w.style().unpolish(w); w.style().polish(w)


class PackageGroup(QtWidgets.QGroupBox):
    def __init__(self, pkg_name: str):
        super().__init__(pkg_name)
        self.setObjectName("pkgGroup")
        self.vbox = QtWidgets.QVBoxLayout(self)
        self.vbox.setSpacing(8)
        self.vbox.setContentsMargins(12, 8, 12, 12)


# ============================== 메인 윈도우 ==============================
class MainWin(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Launch Manager (ROS1)")
        self.resize(1280, 760)
        self.settings = QtCore.QSettings(APP_ORG, APP_NAME)

        # 제한 패키지
        self.limit_packages = None
        pkgs_env = os.environ.get("LAUNCH_PACKAGES")
        if pkgs_env:
            self.limit_packages = {p.strip() for p in pkgs_env.split(",") if p.strip()}
        self.data = {}
        self.cards = {}   # (pkg, rel) -> LaunchItemWidget
        self.procs = {}   # (pkg, rel) -> LaunchProcess
        self.currentKey = None

        # 중앙 스플리터 (좌/우)
        mainSplit = QtWidgets.QSplitter()
        mainSplit.setHandleWidth(8)

        # 좌측: 패키지 그룹 + 카드 리스트 (스크롤)
        self.leftScroll = QtWidgets.QScrollArea()
        self.leftScroll.setWidgetResizable(True)
        self.leftBody = QtWidgets.QWidget()
        self.leftLayout = QtWidgets.QVBoxLayout(self.leftBody)
        self.leftLayout.setSpacing(10)
        self.leftLayout.setContentsMargins(12, 12, 12, 12)
        self.leftLayout.addStretch(1)
        self.leftScroll.setWidget(self.leftBody)
        mainSplit.addWidget(self.leftScroll)

        # 우측: 수직 스플리터(위=로그, 아래=영상)
        rightSplit = QtWidgets.QSplitter(Qt.Vertical)
        rightSplit.setHandleWidth(8)

        # --- 로그 영역 (위) ---
        logPane = QtWidgets.QWidget()
        logLay  = QtWidgets.QVBoxLayout(logPane)
        self.logTitle = QtWidgets.QLabel("로그")
        self.logTitle.setObjectName("logTitle")
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.log.setPlaceholderText("선택된 런치의 실시간 로그가 표시됩니다.")
        copyBtn = QtWidgets.QPushButton("로그 복사")
        copyBtn.clicked.connect(self.copyLog)
        logLay.addWidget(self.logTitle)
        logLay.addWidget(self.log, 1)
        logLay.addWidget(copyBtn, 0, Qt.AlignRight)
        rightSplit.addWidget(logPane)

        # --- 영상 영역 (아래) ---
        videoPane = QtWidgets.QWidget()
        vLay = QtWidgets.QVBoxLayout(videoPane)

        ctrl = QtWidgets.QHBoxLayout()
        self.topicCombo = QtWidgets.QComboBox()
        self.topicCombo.setEditable(False)
        self.refreshTopicsBtn = QtWidgets.QPushButton("토픽 새로고침")
        self.subscribeBtn = QtWidgets.QPushButton("구독 시작")
        self.subscribeBtn.setCheckable(True)
        ctrl.addWidget(QtWidgets.QLabel("영상 토픽"))
        ctrl.addWidget(self.topicCombo, 1)
        ctrl.addWidget(self.refreshTopicsBtn)
        ctrl.addWidget(self.subscribeBtn)
        vLay.addLayout(ctrl)

        self.videoLabel = QtWidgets.QLabel("영상 미표시")
        self.videoLabel.setAlignment(Qt.AlignCenter)
        self.videoLabel.setMinimumHeight(240)
        self.videoLabel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.videoLabel.setStyleSheet("background:#0b1220; border:1px solid #27324a; border-radius:8px;")
        vLay.addWidget(self.videoLabel, 1)

        self.videoInfo = QtWidgets.QLabel("정지")
        vLay.addWidget(self.videoInfo, 0, Qt.AlignRight)

        rightSplit.addWidget(videoPane)
        rightSplit.setStretchFactor(0, 2)  # 로그가 좀 더 크게
        rightSplit.setStretchFactor(1, 3)  # 영상 더 크게

        mainSplit.addWidget(rightSplit)
        mainSplit.setStretchFactor(0, 3)  # 좌측 카드
        mainSplit.setStretchFactor(1, 4)  # 우측 로그+영상

        self.setCentralWidget(mainSplit)

        # 툴바
        tb = self.addToolBar("main")
        tb.setMovable(False)
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("검색: 패키지/파일")
        self.search.textEdited.connect(self.applyFilter)
        actSearch = QtWidgets.QWidgetAction(self)
        actSearch.setDefaultWidget(self.search)
        tb.addAction(actSearch)

        refresh = QtWidgets.QAction("새로고침", self)
        refresh.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        refresh.triggered.connect(self.reloadData)
        tb.addAction(refresh)

        self.themeAct = QtWidgets.QAction("다크", self, checkable=True)
        self.themeAct.triggered.connect(self.toggleTheme)
        tb.addAction(self.themeAct)

        tb.addSeparator()
        stopAll = QtWidgets.QAction("모두 정지", self)
        stopAll.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        stopAll.triggered.connect(self.stopAll)
        tb.addAction(stopAll)

        ros_master = os.environ.get("ROS_MASTER_URI", "(unset)")
        ros_pkg_path = os.environ.get("ROS_PACKAGE_PATH", "(unset)")
        self.statusBar().showMessage(f"ROS_MASTER_URI={ros_master}  ·  ROS_PACKAGE_PATH={ros_pkg_path}")

        # 데이터/상태
        self.reloadData()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refreshStatuses)
        self.timer.start(500)

        # 테마 로드
        if self.settings.value("dark", "1") == "1":
            self.themeAct.setChecked(True)
            self.applyDark(True)
        else:
            self.applyDark(False)

        # 창 상태 복원
        self.restoreGeometry(self.settings.value("geo", b""))
        self.restoreState(self.settings.value("state", b""))

        # --- ROS 이미지 기능 준비 ---
        self._video_enabled = False
        if ROS_OK and os.environ.get("ROS_MASTER_URI"):
            try:
                if not rospy.core.is_initialized():
                    rospy.init_node("ros_launch_manager_gui", anonymous=True, disable_signals=True)
                self._video_enabled = True
            except Exception:
                self._video_enabled = False
        self._img_sub = None
        if self._video_enabled:
            self._img_sub = RosImageSubscriber()
            self._img_sub.frameReady.connect(self._on_frame)
            self.refreshTopicsBtn.clicked.connect(self._fill_image_topics)
            self.subscribeBtn.toggled.connect(self._toggle_subscribe)
            self._fill_image_topics()
        else:
            self.topicCombo.setEnabled(False)
            self.refreshTopicsBtn.setEnabled(False)
            self.subscribeBtn.setEnabled(False)
            self.videoInfo.setText("ROS 영상 비활성 (rospy/cv_bridge/roscore 확인)")

    # ---------- 데이터 ----------
    def clearLeft(self):
        for i in reversed(range(self.leftLayout.count())):
            w = self.leftLayout.itemAt(i).widget()
            if w: w.deleteLater()
        self.leftLayout.addStretch(1)
        self.cards.clear()

    def reloadData(self):
        try:
            self.data = find_launch_files(self.limit_packages)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "오류", f"런치 파일 검색 실패: {e}")
            return
        self.clearLeft()
        for pkg, launches in sorted(self.data.items(), key=lambda x: x[0].lower()):
            grp = PackageGroup(pkg)
            for rel, _abs in launches:
                card = LaunchItemWidget(pkg, rel)
                card.startClicked.connect(lambda *_, p=pkg, r=rel: self.startLaunch(p, r))
                card.stopClicked.connect(lambda *_,  p=pkg, r=rel: self.stopLaunch(p, r))
                card.mouseDoubleClickEvent = (lambda ev, p=pkg, r=rel: self.toggleLaunch(p, r))
                grp.vbox.addWidget(card)
                self.cards[(pkg, rel)] = card
            self.leftLayout.insertWidget(self.leftLayout.count()-1, grp)
        self.applyFilter()

    def applyFilter(self):
        key = self.search.text().strip().lower()
        for idx in range(self.leftLayout.count()-1):
            grp = self.leftLayout.itemAt(idx).widget()
        #    ↑ 위 두 줄은 유지 (검사용)
            if not isinstance(grp, PackageGroup): 
                continue
            vis_any = False
            for i in range(grp.vbox.count()):
                w = grp.vbox.itemAt(i).widget()
                if not isinstance(w, LaunchItemWidget): 
                    continue
                text = f"{w.pkg}/{w.rel_path}".lower()
                vis = (key in text) if key else True
                w.setVisible(vis)
                vis_any = vis_any or vis
            grp.setVisible(vis_any)

    # ---------- 런치 제어 ----------
    def keyFor(self, pkg, rel): return (pkg, rel)

    def startLaunch(self, pkg, rel):
        k = self.keyFor(pkg, rel)
        p = self.procs.get(k)
        if p and p.is_running():
            QtWidgets.QMessageBox.information(self, "정보", "이미 실행 중입니다.")
            return
        p = LaunchProcess(pkg, rel)
        p.textAppended.connect(lambda t, key=k: self.onLog(key, t))
        p.finished.connect(lambda key=k: self.onFinished(key))
        self.procs[k] = p
        self.currentKey = k
        self.updateLogTitle()
        p.start()
        self.refreshCard(k)

    def stopLaunch(self, pkg, rel):
        k = self.keyFor(pkg, rel)
        p = self.procs.get(k)
        if p and p.is_running():
            p.terminate_gracefully()
        self.refreshCard(k)

    def toggleLaunch(self, pkg, rel):
        k = self.keyFor(pkg, rel)
        p = self.procs.get(k)
        if p and p.is_running():
            self.stopLaunch(pkg, rel)
        else:
            self.startLaunch(pkg, rel)

    def stopAll(self):
        for k, p in list(self.procs.items()):
            if p.is_running():
                p.terminate_gracefully()

    # ---------- 상태/로그 ----------
    def refreshStatuses(self):
        for k in list(self.cards.keys()):
            self.refreshCard(k)

    def refreshCard(self, key):
        card = self.cards.get(key)
        p = self.procs.get(key)
        if not card:
            return
        if p and p.is_running():
            card.setRunning(True, str(p.pid() or ""))
        else:
            card.setRunning(False, "")

    def onLog(self, key, text):
        if self.currentKey == key:
            self.log.moveCursor(QtGui.QTextCursor.End)
            self.log.insertPlainText(text)
            self.log.moveCursor(QtGui.QTextCursor.End)

    def onFinished(self, key):
        self.refreshCard(key)
        if self.currentKey == key:
            self.log.appendPlainText("")

    def copyLog(self):
        self.log.selectAll()
        self.log.copy()
        QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "복사됨")

    def updateLogTitle(self):
        if self.currentKey:
            pkg, rel = self.currentKey
            self.logTitle.setText(f"로그 · {pkg}/{rel}")
        else:
            self.logTitle.setText("로그")

    # ---------- ROS 영상 ----------
    def _fill_image_topics(self):
        self.topicCombo.clear()
        try:
            topics = rospy.get_published_topics()
        except Exception:
            self.topicCombo.addItem("(토픽 조회 실패)")
            return
        img_topics = []
        for name, typ in topics:
            if typ in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
                img_topics.append((name, typ, typ == "sensor_msgs/CompressedImage"))
        if not img_topics:
            self.topicCombo.addItem("(영상 토픽 없음)")
            return
        for name, typ, is_comp in img_topics:
            self.topicCombo.addItem(f"{name}  [{typ}]", (name, is_comp))

    def _toggle_subscribe(self, checked):
        if not self._video_enabled or self._img_sub is None:
            return
        if checked:
            data = self.topicCombo.currentData()
            if not data:
                self.subscribeBtn.setChecked(False)
                return
            name, is_comp = data
            self._img_sub.subscribe(name, is_comp)
            self.subscribeBtn.setText("구독 중지")
            self.videoInfo.setText(f"구독: {name}")
        else:
            self._img_sub.unsubscribe()
            self.subscribeBtn.setText("구독 시작")
            self.videoInfo.setText("정지")

    @QtCore.pyqtSlot(object, int, int, float)
    def _on_frame(self, frame_bgr, w, h, fps):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h_, w_, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w_, h_, ch * w_, QtGui.QImage.Format.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        pix = pix.scaled(self.videoLabel.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.videoLabel.setPixmap(pix)
        self.videoInfo.setText(f"{w}x{h} · {fps:.1f} FPS")

    # ---------- 테마 ----------
    def toggleTheme(self):
        self.applyDark(self.themeAct.isChecked())

    def applyDark(self, enabled: bool):
        self.settings.setValue("dark", "1" if enabled else "0")
        self.setStyleSheet(self.baseStyleSheet(light=not enabled))

    # ---------- 종료 시 상태 저장 ----------
    def closeEvent(self, e):
        self.settings.setValue("geo", self.saveGeometry())
        self.settings.setValue("state", self.saveState())
        try:
            if self._img_sub:
                self._img_sub.unsubscribe()
        except Exception:
            pass
        super().closeEvent(e)

    # ---------- 스타일시트 ----------
    def baseStyleSheet(self, light=False):
        if light:
            bg = "#fafafa"; fg = "#1f2937"; card="#ffffff"; border="#e5e7eb"
            badgeRun="#16a34a"; badgeStop="#9ca3af"; group="#111827"
        else:
            bg = "#0f172a"; fg = "#e5e7eb"; card="#111827"; border="#27324a"
            badgeRun="#22c55e"; badgeStop="#64748b"; group="#cbd5e1"
        return f"""
        QMainWindow {{ background: {bg}; color: {fg}; }}
        QLabel, QPushButton, QToolButton, QLineEdit, QGroupBox, QPlainTextEdit {{
            color: {fg}; font-size: 14px;
        }}
        QToolBar {{ background: transparent; border: none; padding: 6px; }}
        QLineEdit {{
            background: {card}; border: 1px solid {border};
            border-radius: 8px; padding: 6px 8px; min-width: 260px;
        }}
        QGroupBox#pkgGroup {{
            border: 1px solid {border}; border-radius: 12px; margin-top: 16px; padding-top: 12px;
            background: {card}; font-weight: 600; color: {group};
        }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 12px; top: 6px; }}
        #card {{
            background: {card}; border: 1px solid {border};
            border-radius: 12px; padding: 8px 10px;
        }}
        #itemTitle {{ font-weight: 600; }}
        #pidLabel {{ color: {badgeStop}; }}
        #badgeRunning {{
            background: {badgeRun}; color: black; border-radius: 8px; padding: 2px 8px; min-height:18px;
        }}
        #badgeStopped {{
            background: {badgeStop}; color: black; border-radius: 8px; padding: 2px 8px; min-height:18px;
        }}
        QPlainTextEdit {{
            background: {card}; border: 1px solid {border}; border-radius: 8px;
        }}
        QPushButton, QToolButton {{
            background: transparent; border: 1px solid {border};
            border-radius: 8px; padding: 6px 10px;
        }}
        QPushButton:hover, QToolButton:hover {{ border-color: #3b82f6; }}
        QSplitter::handle {{ background: {border}; }}
        """


def main():
    if not os.environ.get("ROS_MASTER_URI"):
        print("[경고] ROS_MASTER_URI가 없습니다. roscore 또는 환경 설정이 필요합니다.")
    if not os.environ.get("ROS_PACKAGE_PATH"):
        print("[경고] ROS_PACKAGE_PATH가 비어 있습니다. catkin/devel/setup.bash 를 source 하세요.")

    app = QtWidgets.QApplication(sys.argv)
    app.setOrganizationName(APP_ORG)
    app.setApplicationName(APP_NAME)
    win = MainWin()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
