#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, cv2, numpy as np, argparse, threading, signal, subprocess, sys
from datetime import datetime
import rospy

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from std_msgs.msg import Int8, String
from cv_bridge import CvBridge, CvBridgeError

# OpenCV 내부 스레드/OPENCL 비활성화(종료 충돌 예방)
try: cv2.setNumThreads(1)
except Exception: pass
try: cv2.ocl.setUseOpenCL(False)
except Exception: pass

def remove_ros_args(args):
    non_ros = [args[0]]
    for a in args[1:]:
        if not (a.startswith('__') or ':' in a):
            non_ros.append(a)
    return non_ros

def parse_args():
    filtered = remove_ros_args(sys.argv)[1:]
    p = argparse.ArgumentParser("Visualize YOLO & cmd_vel, publish/record annotated video")
    p.add_argument("--image-topic", default="/input_image")
    p.add_argument("--yolo-topic",  default="/YoloInfo")
    p.add_argument("--yolo-state",  default="/yolo_state")
    p.add_argument("--cmd-topic",   default="/cmd_vel/auto")

    # 출력 토픽 (raw+compressed 동시 지원)
    p.add_argument("--out-topic",        default="/yolo/annotated")
    p.add_argument("--out-topic-comp",   default="/yolo/annotated/compressed")
    p.add_argument("--jpeg-quality", type=int, default=85, help="0~100")

    p.add_argument("--pub-fps", type=float, default=30.0, help="output publish fps limit")
    p.add_argument("--draw-only-ball", action="store_true")
    p.add_argument("--record", action="store_true")
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_dir = os.path.expanduser("./video"); os.makedirs(default_dir, exist_ok=True)
    p.add_argument("--record-path", default=os.path.join(default_dir, f"{ts}.mp4"))
    p.add_argument("--record-fps", type=float, default=45.0)
    p.add_argument("--timeout-yolo", type=float, default=0.5)
    p.add_argument("--timeout-cmd",  type=float, default=1.0)
    p.add_argument("--display", action="store_true")

    # 우측 상단 표시용 인자 추가 (ROS 파라미터가 없으면 기본값으로 사용)
    p.add_argument("--mode-display-text", default="", help="Text to display on top right corner (e.g., 'person Finding' or 'Goal Kick')")

    return p.parse_args(args=filtered)

# ---------- FFmpeg 파이프 라이터 ----------
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
    def release(self, timeout=5): # timeout 인자를 추가하여 안정성 확보
        if not self.proc: return
        try:
            if self.proc.stdin:
                try: self.proc.stdin.flush()
                except Exception: pass
                self.proc.stdin.close()
            # wait에 timeout 적용
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            rospy.logwarn("FFmpeg process did not terminate within timeout. Forcing kill.")
            try: self.proc.kill()
            except Exception: pass
        except Exception:
            try: self.proc.kill()
            except Exception: pass
        finally:
            self.proc=None

class Viz:
    def __init__(self, args):
        rospy.init_node("visualize_person_tracking", anonymous=False)
        self.args = args
        
        # ROS 파라미터에서 mode-display-text 읽기
        self.mode_display_text = rospy.get_param('~mode-display-text', args.mode_display_text)
        
        # 퍼블리셔 (raw + compressed)
        self.bridge = CvBridge()
        self.pub_raw  = rospy.Publisher(args.out_topic,      Image,           queue_size=1)
        self.pub_comp = rospy.Publisher(args.out_topic_comp, CompressedImage, queue_size=1)

        # 입력 토픽 구독
        image_topic = args.image_topic
        self._raw_input = not image_topic.endswith("/compressed")
        if self._raw_input:
            rospy.Subscriber(image_topic, Image,          self.image_cb_raw,        queue_size=1, buff_size=2**22)
            rospy.loginfo("Subscribing RAW: %s", image_topic)
        else:
            rospy.Subscriber(image_topic, CompressedImage, self.image_cb_compressed, queue_size=1, buff_size=2**22)
            rospy.loginfo("Subscribing COMPRESSED: %s", image_topic)

        # pi_mode는 String 토픽 구독
        rospy.Subscriber("/pi_mode",     String, self.pi_mode_cb, queue_size=10)
        rospy.Subscriber(args.yolo_topic, Yolo,   self.yolo_cb,   queue_size=5)
        rospy.Subscriber(args.cmd_topic,  Twist,  self.cmd_cb,    queue_size=5)
        rospy.Subscriber(args.yolo_state, Int8,   self.state_cb,  queue_size=10)

        # 상태
        self.last_dets=[]; self.t_last_yolo=None
        self.state=0; self.t_last_state=None
        self.v=0.0; self.y=0.0; self.w=0.0

        # pi_mode 표시용
        self.pi_mode_text = ""           # 최신 모드 문자열
        self.t_last_mode  = None         # 수신 시각

        self.state_map={0:"go",1:"search left",2:"search right",3:"calib",4:"hold"}

        # 퍼블리시 FPS 제한
        self._last_pub_ts = rospy.Time(0)
        self._pub_period  = rospy.Duration(0 if args.pub_fps<=0 else 1.0/args.pub_fps)

        # 녹화
        self.writer=None; self._written_frames=0; self._lock=threading.Lock(); self._alive=True

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        rospy.loginfo("visualize_person_tracking started")
        rospy.loginfo("  in=%s  yolo=%s  state=%s  cmd=%s", image_topic, args.yolo_topic, args.yolo_state, args.cmd_topic)
        rospy.loginfo("  out(raw)=%s  out(comp)=%s  pub_fps=%.1f  jpeg=%d",
                      args.out_topic, args.out_topic_comp, args.pub_fps, args.jpeg_quality)
        # mode_display_text 로깅
        if self.mode_display_text: 
             rospy.loginfo("  Top right text: %s (from param)", self.mode_display_text)
        elif args.mode_display_text:
             rospy.loginfo("  Top right text: %s (from args)", args.mode_display_text)

    def pi_mode_cb(self, msg: String):
        # 최신 모드 문자열과 수신 시각 저장
        try:
            self.pi_mode_text = str(msg.data)
        except Exception:
            self.pi_mode_text = ""
        self.t_last_mode = rospy.Time.now()

    def _signal_handler(self, signum, _):
        self._alive=False
        try: rospy.signal_shutdown(f"signal {signum}")
        except Exception: pass

    def on_shutdown(self):
        self._alive=False
        with self._lock:
            if self.writer:
                try:
                    # FFmpegWriter release에 timeout 적용
                    self.writer.release(timeout=5) 
                    rospy.loginfo("FFmpegWriter released (%d frames) -> %s", self._written_frames, self.args.record_path)
                except Exception as e:
                    rospy.logerr("FFmpegWriter release error: %s", e)
                self.writer=None

    # --------- Subscribers ----------
    def state_cb(self, msg: Int8):
        self.state=int(msg.data); self.t_last_state=rospy.Time.now()

    def yolo_cb(self, msg: Yolo):
        if not self._alive:
            return

        now = rospy.Time.now()
        dets = []

        for d in msg.detections:
            label = (d.label or "").strip().lower()
            # 🔥 라벨이 person 아닌 건 전부 무시
            if label != "person":
                continue

            dets.append(dict(
                xc   = int(d.x_center),
                yc   = int(d.y_center),
                w    = int(getattr(d, "w", 0)),
                h    = int(getattr(d, "h", 0)),
                label= d.label if d.label else "person",
                conf = float(getattr(d, "conf", 0.0)),
            ))

        self.last_dets = dets
        self.t_last_yolo = now


    def cmd_cb(self, msg: Twist):
        if not self._alive: return
        self.v=msg.linear.x
        self.y=msg.linear.y
        self.w=msg.angular.z

    def image_cb_raw(self, msg: Image):
        if not self._alive: return
        try: frame=self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError: return
        self._handle_frame(frame, msg.header)

    def image_cb_compressed(self, msg: CompressedImage):
        if not self._alive: return
        try:
            np_arr=np.frombuffer(msg.data, np.uint8)
            frame=cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception: return
        if frame is None: return
        self._handle_frame(frame, msg.header)

    # --------- 공통 처리 ---------
    def _ensure_writer(self, frame):
        if not self.args.record: return
        with self._lock:
            if self.writer: return
            path=self.args.record_path
            if os.path.isdir(path):
                ts=datetime.now().strftime('%Y%m%d_%H%M%S')
                path=os.path.join(path, f"{ts}.mp4")
            h,w=frame.shape[:2]
            try:
                self.writer=FFmpegWriter(path, self.args.record_fps, w, h)
                rospy.loginfo("Recording via ffmpeg -> %s (%dx%d @ %.1ffps)", path, w, h, self.args.record_fps)
            except Exception as e:
                self.writer=None; self.args.record=False
                rospy.logerr("FFmpeg init failed: %s (record disabled)", e)

    def _fix_header(self, header):
        # header가 비어 있으면 기본값 세팅
        h = type(header)()
        h.stamp = header.stamp if header and header.stamp!=rospy.Time() else rospy.Time.now()
        h.frame_id = header.frame_id if header and header.frame_id else "camera"
        h.seq = header.seq if header and hasattr(header,"seq") else 0
        return h

    def _handle_frame(self, frame, header):
        # 퍼블리시 FPS 제한
        now = rospy.Time.now()
        if self._pub_period.to_sec()>0 and (now - self._last_pub_ts) < self._pub_period:
            return
        self._last_pub_ts = now

        H,W = frame.shape[:2]
        IS_HOLD = (self.state == 4)

        # 중심 표시 (노란 십자)
        cv2.drawMarker(frame, (W//2, H//2), (0,255,255),
                       markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

        # 좌상단: pi_mode 표시 
        mode_age = (now - self.t_last_mode).to_sec() if self.t_last_mode else None
        
        # 데이터가 오래되었거나 없는 경우
        is_mode_old = mode_age is None or mode_age > self.args.timeout_cmd
        
        current_display_txt = self.pi_mode_text
        if is_mode_old:
            current_display_txt = "finding..." 
        
        # 좌상단 텍스트 출력 (예: person chase, finding...)
        if current_display_txt and current_display_txt != "avoid wall":
            cv2.putText(frame, current_display_txt, (8,35),
                        cv2.FONT_HERSHEY_COMPLEX, 1.3,
                        (255,255,255), 2, cv2.LINE_AA)
        
        # 우측 상단: launch 파라미터 텍스트 표시
        if self.mode_display_text:
            text_to_display = self.mode_display_text
            font = cv2.FONT_HERSHEY_COMPLEX
            font_scale = 1.3
            font_thickness = 2
            color = (0, 255, 255) # 노란색
            
            # 텍스트 크기 계산
            text_size = cv2.getTextSize(text_to_display, font, font_scale, font_thickness)[0]
            text_w, text_h = text_size
            
            # 텍스트 위치 계산 (우측 상단)
            text_x = W - text_w - 8
            text_y = 35
            
            cv2.putText(frame, text_to_display, (text_x, text_y), 
                        font, font_scale, color, font_thickness, cv2.LINE_AA)

        # ⭐️ 중앙: 'avoid wall' 메시지 표시 (수정된 로직) ⭐️
        # pi_mode_text가 정확히 "avoid wall"이고, 데이터가 오래되지 않았을 때만 메시지를 표시합니다.
        if self.pi_mode_text == "avoid wall" and not is_mode_old:
            text_to_display = "AVOID OBSTACLE"
            font = cv2.FONT_HERSHEY_COMPLEX
            font_scale = 2.0  # 글자 크기
            font_thickness = 4 # 글자 두께
            text_color = (0, 0, 255) # 빨간색 (BGR)
            background_color = (0, 0, 0) # 검은색 (BGR)

            # 텍스트 크기 계산
            text_size, baseline = cv2.getTextSize(text_to_display, font, font_scale, font_thickness)
            text_w, text_h = text_size
            
            # 텍스트 중앙 위치 계산
            text_x = int((W - text_w) / 2)
            text_y = int(H / 2 + (text_h + baseline) / 2) 

            # 텍스트 배경 직사각형의 좌표 계산
            padding = 10 
            rect_x1 = text_x - padding
            rect_y1 = text_y - text_h - padding
            rect_x2 = text_x + text_w + padding
            rect_y2 = text_y + baseline + padding

            cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2),
                          background_color, -1)

            cv2.putText(frame, text_to_display, (text_x, text_y),
                        font, font_scale, text_color, font_thickness, cv2.LINE_AA)
        
        # YOLO 박스 (현재 last_dets에 들어있는 것들 — 여기서는 person만)
        fresh_yolo = self.t_last_yolo and (now - self.t_last_yolo).to_sec() <= self.args.timeout_yolo
        if fresh_yolo:
            for d in self.last_dets:
                x1=int(d["xc"]-d["w"]/2); y1=int(d["yc"]-d["h"]/2)
                x2=int(d["xc"]+d["w"]/2); y2=int(d["yc"]+d["h"]/2)
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,220,0),2)
                label=f"{d['label']} {d['conf']:.2f}"
                cv2.putText(frame,label,(x1,max(0,y1-6)),
                            cv2.FONT_HERSHEY_COMPLEX,0.6,(0,220,0),2,cv2.LINE_AA)
                cv2.circle(frame,(d["xc"],d["yc"]),3,(0,220,0),-1)

        # 헤더 보정
        hdr = self._fix_header(header)

        # raw publish
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header = hdr
            self.pub_raw.publish(img_msg)
        except Exception as e:
            rospy.logerr_throttle(2.0, "raw publish failed: %s", e)

        # compressed publish
        try:
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)])
            if ok:
                comp = CompressedImage(); comp.header = hdr; comp.format = "jpeg"; comp.data = enc.tobytes()
                self.pub_comp.publish(comp)
        except Exception as e:
            rospy.logerr_throttle(2.0, "compressed publish failed: %s", e)

        # (옵션) 로컬 디스플레이
        if self.args.display:
            try: 
                window_name = "annotated"
                window_width = 1600
                window_height = 900
                
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

                cv2.resizeWindow(window_name, window_width, window_height)
                cv2.moveWindow(window_name, 0, 0)
                
                cv2.imshow(window_name, frame) 
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
        # 프로그램 종료 시 모든 OpenCV 창 닫기
        cv2.destroyAllWindows() 
        viz.on_shutdown()

if __name__ == "__main__":
    main()
