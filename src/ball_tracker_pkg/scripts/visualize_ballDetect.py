#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import argparse
from datetime import datetime
import rospy
import threading
import signal
import subprocess

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError

# OpenCV 내부 스레드/OPENCL 비활성화(종료 시 충돌 완화)
try:
    cv2.setNumThreads(1)
except Exception:
    pass
try:
    cv2.ocl.setUseOpenCL(False)
except Exception:
    pass


def parse_args():
    p = argparse.ArgumentParser(
        description="Visualize YOLO boxes; overlay bbox center & size at TOP of the frame; optionally record."
    )
    p.add_argument("--image-topic", default="/usb_cam/image_raw/compressed")
    p.add_argument("--yolo-topic", default="/YoloInfo")

    # optional: 주지 않으면 구독 안 함(화면에도 렌더링 안 함)
    p.add_argument("--yolo-state", default=None, help="(optional) state topic; not rendered")
    p.add_argument("--cmd-topic", default=None, help="(optional) cmd_vel topic; not rendered")

    p.add_argument("--out-topic", default="/yolo/annotated/compressed")
    p.add_argument("--draw-only-ball", action="store_true", help="draw only detections with label 'Ball'")
    p.add_argument("--record", action="store_true", help="save annotated video to file")

    # 기본 저장 경로: ./video/년월일시분초.mp4 (폴더 자동 생성)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_dir = os.path.expanduser("./video")
    os.makedirs(default_dir, exist_ok=True)
    default_record_path = os.path.join(default_dir, f"{ts}.mp4")
    p.add_argument("--record-path", default=default_record_path)

    p.add_argument("--record-fps", type=float, default=45.0)
    p.add_argument("--timeout-yolo", type=float, default=0.5, help="seconds until YOLO considered stale")

    # 디스플레이는 충돌 원인이라 기본 미사용 권장
    p.add_argument("--display", action="store_true", help="show a window (cv2.imshow)")
    return p.parse_args()


# ---------- FFmpeg 파이프 라이터 ----------
class FFmpegWriter:
    def __init__(self, path, fps, width, height):
        self.path = path
        self.proc = None
        # raw BGR24 → libx264 yuv420p mp4
        cmd = [
            "ffmpeg",
            "-loglevel", "error",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "-",  # stdin
            "-an",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            path,
        ]
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except Exception as e:
            self.proc = None
            raise RuntimeError(f"ffmpeg start failed: {e}")

    def write(self, frame_bgr):
        if self.proc is None or self.proc.stdin is None:
            return
        self.proc.stdin.write(frame_bgr.tobytes())

    def release(self):
        if self.proc is None:
            return
        try:
            if self.proc.stdin:
                try:
                    self.proc.stdin.flush()
                except Exception:
                    pass
                self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        finally:
            self.proc = None


def put_text_with_bg(img, text, org, font_scale=0.6, thickness=2,
                     text_color=(0, 0, 0), bg_color=(255, 255, 255), alpha=0.65):
    """
    상단 고정 오버레이 가독성을 위해 반투명 배경 박스 + 텍스트 출력
    org: (x,y) - baseline 좌측
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), bl = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    pad = 4
    x1, y1 = x - pad, y - th - pad
    x2, y2 = x + tw + pad, y + bl + pad
    x1, y1 = max(0, x1), max(0, y1)
    overlay = img.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), bg_color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    cv2.putText(img, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)
    return th + bl + pad * 2  # box height to step next line


class Viz:
    def __init__(self, args):
        rospy.init_node("visualize_ball_tracking", anonymous=False)

        self.args = args
        self.pub = rospy.Publisher(args.out_topic, CompressedImage, queue_size=1)

        # 입력 이미지: /compressed로 끝나면 CompressedImage, 아니면 Image
        if args.image_topic.endswith("/compressed"):
            rospy.Subscriber(args.image_topic, CompressedImage, self.image_cb_compressed, queue_size=1, buff_size=2**22)
            self.bridge = None
            rospy.loginfo("Subscribing COMPRESSED: %s", args.image_topic)
        else:
            self.bridge = CvBridge()
            rospy.Subscriber(args.image_topic, Image, self.image_cb_raw, queue_size=1, buff_size=2**22)
            rospy.loginfo("Subscribing RAW: %s", args.image_topic)

        rospy.Subscriber(args.yolo_topic, Yolo, self.yolo_cb, queue_size=5)

        # optional 구독들(렌더링 X)
        if args.cmd_topic:
            rospy.Subscriber(args.cmd_topic, Twist, self.cmd_cb, queue_size=5)
            rospy.loginfo("Subscribing cmd_topic = %s (not rendered)", args.cmd_topic)
        else:
            rospy.loginfo("cmd_topic disabled (not subscribed)")

        if args.yolo_state:
            rospy.Subscriber(args.yolo_state, Int8, self.state_cb, queue_size=10)
            rospy.loginfo("Subscribing yolo_state = %s (not rendered)", args.yolo_state)
        else:
            rospy.loginfo("yolo_state disabled (not subscribed)")

        self.last_dets = []  # list of dict(xc,yc,w,h,label,conf)
        self.t_last_yolo = None

        # (유지) 구독만 하고 사용하지 않음
        self.last_cmd = None
        self.state = 0

        # 녹화기/상태
        self.writer = None  # FFmpegWriter
        self._written_frames = 0
        self._lock = threading.Lock()
        self._alive = True

        # 종료 핸들링
        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        rospy.loginfo("visualize_ball_tracking started:")
        rospy.loginfo("  yolo_topic =%s", args.yolo_topic)
        rospy.loginfo("  out_topic  =%s", args.out_topic)
        rospy.loginfo("  record=%s path=%s fps=%.1f", args.record, args.record_path, args.record_fps)

    def _signal_handler(self, signum, frame):
        self._alive = False
        try:
            rospy.signal_shutdown(f"signal {signum}")
        except Exception:
            pass

    def on_shutdown(self):
        self._alive = False
        with self._lock:
            if self.writer is not None:
                try:
                    self.writer.release()
                    rospy.loginfo("FFmpegWriter released (%d frames) -> %s", self._written_frames, self.args.record_path)
                except Exception as e:
                    rospy.logerr("FFmpegWriter release error: %s", e)
                self.writer = None

    # --------- Subscribers ----------
    def state_cb(self, msg: Int8):
        self.state = int(msg.data)  # not rendered

    def yolo_cb(self, msg: Yolo):
        if not self._alive:
            return
        now = rospy.Time.now()
        dets = []
        for d in msg.detections:
            if self.args.draw_only_ball and (d.label or "").lower() != "ball":
                continue
            dets.append(
                dict(
                    xc=int(d.x_center),
                    yc=int(d.y_center),
                    w=int(getattr(d, "w", 0)),
                    h=int(getattr(d, "h", 0)),
                    label=d.label if d.label else "Ball",
                    conf=float(getattr(d, "conf", 0.0)),
                )
            )
        self.last_dets = dets
        self.t_last_yolo = now

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg  # not rendered

    # RAW 이미지 콜백
    def image_cb_raw(self, msg: Image):
        if not self._alive:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return
        self._handle_frame(frame, msg.header)

    # COMPRESSED 이미지 콜백
    def image_cb_compressed(self, msg: CompressedImage):
        if not self._alive:
            return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is None:
            return
        self._handle_frame(frame, msg.header)

    # --------- 공통 처리 ---------
    def _ensure_writer(self, frame):
        if not self.args.record:
            return
        with self._lock:
            if self.writer is not None:
                return
            h, w = frame.shape[:2]
            try:
                self.writer = FFmpegWriter(self.args.record_path, self.args.record_fps, w, h)
                rospy.loginfo("Recording via ffmpeg to %s (%dx%d @ %.1ffps)", self.args.record_path, w, h, self.args.record_fps)
            except Exception as e:
                self.writer = None
                self.args.record = False
                rospy.logerr("FFmpeg init failed, disable recording: %s", e)

    def _handle_frame(self, frame, header):
        H, W = frame.shape[:2]

        # 화면 중앙 십자선
        cx, cy = W // 2, H // 2
        cv2.drawMarker(frame, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

        # YOLO 박스(신선할 때만)
        fresh_yolo = self.t_last_yolo and (rospy.Time.now() - self.t_last_yolo).to_sec() <= self.args.timeout_yolo
        overlay_lines = []  # 상단에 쌓아 표시할 문자열들
        if fresh_yolo:
            img_area = float(W * H) if (W > 0 and H > 0) else 1.0
            for i, d in enumerate(self.last_dets, start=1):
                x1 = int(d["xc"] - d["w"] / 2)
                y1 = int(d["yc"] - d["h"] / 2)
                x2 = int(d["xc"] + d["w"] / 2)
                y2 = int(d["yc"] + d["h"] / 2)

                # bbox/중심점 자체는 그대로 그림
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
                cv2.circle(frame, (d["xc"], d["yc"]), 3, (0, 220, 0), -1)

                # 상단 오버레이용 문자열 구성
                area = max(0, int(d["w"]) * int(d["h"]))
                ratio = area / img_area  # 0~1
                line1 = f"[{i}] {d['label']} {d['conf']:.2f}"
                line2 = f"    center=({d['xc']},{d['yc']})  size={area}px  ratio={ratio:.3f}"
                overlay_lines.append(line1)
                overlay_lines.append(line2)
        else:
            overlay_lines = ["YOLO: stale/no data"]

        # ===== 상단(고정)으로 텍스트 렌더링 =====
        # 시작 위치와 줄간격 설정
        base_x, base_y = 8, 24  # 첫 줄 기준 (왼쪽 상단)
        step = 0
        for line in overlay_lines:
            # 밝은 배경 + 짙은 글자 (반전 대비)
            h = put_text_with_bg(
                frame, line, (base_x, base_y + step),
                font_scale=0.7, thickness=2,
                text_color=(20, 20, 20), bg_color=(220, 255, 220), alpha=0.75
            )
            step += max(h, 22)  # 줄간격

        # 퍼블리시(CompressedImage)
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok and self._alive:
            out = CompressedImage()
            out.header = header
            out.format = "jpeg"
            out.data = enc.tobytes()
            self.pub.publish(out)

        # 녹화
        if self.args.record and self._alive:
            self._ensure_writer(frame)
            with self._lock:
                if self.writer is not None:
                    try:
                        self.writer.write(frame)
                        self._written_frames += 1
                    except Exception as e:
                        rospy.logerr_throttle(2.0, "ffmpeg write error: %s", e)

        # (디스플레이는 비권장)
        if self.args.display:
            try:
                cv2.imshow("annotated", frame)
                cv2.waitKey(1)
            except Exception:
                pass


def main():
    args = parse_args()
    viz = Viz(args)
    rate = rospy.Rate(200)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    finally:
        viz.on_shutdown()


if __name__ == "__main__":
    main()
