#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ✅ 녹화 기능은 유지하되, 현재는 모두 주석 처리 (display only)

import os, cv2, numpy as np, argparse, threading, signal, subprocess, sys
from datetime import datetime
import rospy

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from std_msgs.msg import Int8, String
from cv_bridge import CvBridge, CvBridgeError

# OpenCV 내부 스레드/OPENCL 비활성화
try:
    cv2.setNumThreads(1)
except Exception:
    pass
try:
    cv2.ocl.setUseOpenCL(False)
except Exception:
    pass


def remove_ros_args(args):
    non_ros = [args[0]]
    for a in args[1:]:
        if not (a.startswith("__") or ":" in a):
            non_ros.append(a)
    return non_ros


def parse_args():
    filtered = remove_ros_args(sys.argv)[1:]
    p = argparse.ArgumentParser("Visualize YOLO & cmd_vel (display only for now)")
    p.add_argument("--image-topic", default="/input_image")
    p.add_argument("--yolo-topic", default="/YoloInfo")
    p.add_argument("--yolo-state", default="/yolo_state")
    p.add_argument("--cmd-topic", default="/cmd_vel/auto")
    p.add_argument("--out-topic", default="/yolo/annotated")
    p.add_argument("--out-topic-comp", default="/yolo/annotated/compressed")
    p.add_argument("--jpeg-quality", type=int, default=85)

    p.add_argument("--pub-fps", type=float, default=30.0)
    p.add_argument("--draw-only-ball", action="store_true")
    p.add_argument("--timeout-yolo", type=float, default=0.5)
    p.add_argument("--timeout-cmd", type=float, default=1.0)
    p.add_argument("--display", action="store_true", default=True)
    p.add_argument("--mode-display-text", default="")

    # ✅ 녹화 관련 인자는 그대로 두되, 기능은 주석 처리함
    # p.add_argument("--record", action="store_true")
    # ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    # default_dir = os.path.expanduser("./video"); os.makedirs(default_dir, exist_ok=True)
    # p.add_argument("--record-path", default=os.path.join(default_dir, f"{ts}.mp4"))
    # p.add_argument("--record-fps", type=float, default=45.0)

    return p.parse_args(args=filtered)


# ---------- FFmpeg 파이프 라이터 ----------
# ✅ 임시 비활성화
"""
class FFmpegWriter:
    def __init__(self, path, fps, width, height):
        self.proc = None; self.path = path
        cmd = ["ffmpeg","-loglevel","error","-y","-f","rawvideo","-pix_fmt","bgr24",
               "-s",f"{width}x{height}","-r",str(fps),"-i","-","-an",
               "-c:v","libx264","-preset","veryfast","-pix_fmt","yuv420p","-movflags","+faststart",path]
        try: self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except Exception as e:
            raise RuntimeError(f"ffmpeg start failed: {e}")

    def write(self, frame_bgr):
        if self.proc and self.proc.stdin:
            self.proc.stdin.write(frame_bgr.tobytes())

    def release(self, timeout=5):
        if not self.proc: return
        try:
            if self.proc.stdin:
                try: self.proc.stdin.flush()
                except Exception: pass
                self.proc.stdin.close()
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            rospy.logwarn("FFmpeg process did not terminate. Forcing kill.")
            try: self.proc.kill()
            except Exception: pass
        except Exception:
            try: self.proc.kill()
            except Exception: pass
        finally:
            self.proc=None
"""


class Viz:
    def __init__(self, args):
        rospy.init_node("visualize_ball_tracking", anonymous=False)
        self.args = args
        self.mode_display_text = rospy.get_param("~mode-display-text", args.mode_display_text)

        self.bridge = CvBridge()
        self.pub_raw = rospy.Publisher(args.out_topic, Image, queue_size=1)
        self.pub_comp = rospy.Publisher(args.out_topic_comp, CompressedImage, queue_size=1)

        # 입력 이미지 구독
        image_topic = args.image_topic
        self._raw_input = not image_topic.endswith("/compressed")
        if self._raw_input:
            rospy.Subscriber(image_topic, Image, self.image_cb_raw, queue_size=1, buff_size=2**22)
        else:
            rospy.Subscriber(image_topic, CompressedImage, self.image_cb_compressed, queue_size=1, buff_size=2**22)

        rospy.Subscriber("/pi_mode", String, self.pi_mode_cb, queue_size=10)
        rospy.Subscriber(args.yolo_topic, Yolo, self.yolo_cb, queue_size=5)
        rospy.Subscriber(args.cmd_topic, Twist, self.cmd_cb, queue_size=5)
        rospy.Subscriber(args.yolo_state, Int8, self.state_cb, queue_size=10)

        # 상태 변수
        self.last_dets = []
        self.t_last_yolo = None
        self.state = 0
        self.t_last_state = None
        self.v = 0.0
        self.y = 0.0
        self.w = 0.0
        self.pi_mode_text = ""
        self.t_last_mode = None
        self.state_map = {0: "go", 1: "search left", 2: "search right", 3: "calib", 4: "hold"}

        self._last_pub_ts = rospy.Time(0)
        self._pub_period = rospy.Duration(0 if args.pub_fps <= 0 else 1.0 / args.pub_fps)
        self._lock = threading.Lock()
        self._alive = True

        # ✅ 녹화 관련 속성 주석 처리
        # self.writer=None; self._written_frames=0

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        rospy.loginfo("✅ visualize_ball_tracking started (Recording disabled, Display only)")

    def pi_mode_cb(self, msg: String):
        try:
            self.pi_mode_text = str(msg.data)
        except Exception:
            self.pi_mode_text = ""
        self.t_last_mode = rospy.Time.now()

    def _signal_handler(self, signum, _):
        self._alive = False
        try:
            rospy.signal_shutdown(f"signal {signum}")
        except Exception:
            pass

    def on_shutdown(self):
        self._alive = False
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        # ✅ 녹화 종료도 주석 처리
        # with self._lock:
        #     if self.writer:
        #         try:
        #             self.writer.release(timeout=5)
        #             rospy.loginfo("FFmpegWriter released (%d frames) -> %s", self._written_frames, self.args.record_path)
        #         except Exception as e:
        #             rospy.logerr("FFmpeg release error: %s", e)
        #         self.writer=None

    # --------- Subscribers ----------
    def state_cb(self, msg: Int8):
        self.state = int(msg.data)
        self.t_last_state = rospy.Time.now()

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
        if not self._alive:
            return
        self.v = msg.linear.x
        self.y = msg.linear.y
        self.w = msg.angular.z

    def image_cb_raw(self, msg: Image):
        if not self._alive:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return
        self._handle_frame(frame, msg.header)

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

    def _fix_header(self, header):
        h = type(header)()
        h.stamp = header.stamp if header and header.stamp != rospy.Time() else rospy.Time.now()
        h.frame_id = header.frame_id if header and header.frame_id else "camera"
        h.seq = header.seq if header and hasattr(header, "seq") else 0
        return h

    def _handle_frame(self, frame, header):
        now = rospy.Time.now()
        if self._pub_period.to_sec() > 0 and (now - self._last_pub_ts) < self._pub_period:
            return
        self._last_pub_ts = now

        H, W = frame.shape[:2]

        # 중심 십자 표시
        cv2.drawMarker(frame, (W // 2, H // 2), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

        # pi_mode 텍스트
        mode_age = (now - self.t_last_mode).to_sec() if self.t_last_mode else None
        is_mode_old = mode_age is None or mode_age > self.args.timeout_cmd
        text = self.pi_mode_text if not is_mode_old else "finding..."
        if text and text != "avoid wall":
            cv2.putText(frame, text, (8, 35), cv2.FONT_HERSHEY_COMPLEX, 1.3, (255, 255, 255), 2, cv2.LINE_AA)

        # 우측 상단 텍스트
        if self.mode_display_text:
            t = self.mode_display_text
            s = cv2.getTextSize(t, cv2.FONT_HERSHEY_COMPLEX, 1.3, 2)[0]
            cv2.putText(frame, t, (W - s[0] - 8, 35), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 255), 2, cv2.LINE_AA)

        # YOLO 박스
        fresh_yolo = self.t_last_yolo and (now - self.t_last_yolo).to_sec() <= self.args.timeout_yolo
        if fresh_yolo:
            for d in self.last_dets:
                x1 = int(d["xc"] - d["w"] / 2)
                y1 = int(d["yc"] - d["h"] / 2)
                x2 = int(d["xc"] + d["w"] / 2)
                y2 = int(d["yc"] + d["h"] / 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
                label = f"{d['label']} {d['conf']:.2f}"
                cv2.putText(frame, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 220, 0), 2, cv2.LINE_AA)

        # 퍼블리시
        hdr = self._fix_header(header)
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header = hdr
            self.pub_raw.publish(msg)
        except Exception:
            pass
        try:
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)])
            if ok:
                comp = CompressedImage()
                comp.header = hdr
                comp.format = "jpeg"
                comp.data = enc.tobytes()
                self.pub_comp.publish(comp)
        except Exception:
            pass

        # ✅ 녹화 코드 전부 주석 처리
        """
        if self.args.record and self._alive:
            self._ensure_writer(frame)
            with self._lock:
                if self.writer:
                    try: self.writer.write(frame); self._written_frames += 1
                    except Exception as e: rospy.logerr_throttle(2.0, "ffmpeg write error: %s", e)
        """

        # 디스플레이
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
        cv2.destroyAllWindows()
        viz.on_shutdown()


if __name__ == "__main__":
    main()
