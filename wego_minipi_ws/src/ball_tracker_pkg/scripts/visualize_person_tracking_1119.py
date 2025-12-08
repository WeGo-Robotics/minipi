#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from yolo11_detect_pkg.msg import Yolo
from std_msgs.msg import Int8, String


class VisualizePersonTracking:
    def __init__(self):
        rospy.init_node("visualize_person_tracking", anonymous=False)

        # ---------- 파라미터 ----------
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo/detections")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel/auto")
        self.state_topic = rospy.get_param("~state_topic", "/ball/state")    # 이름만 ball인데 아무 영향 없음
        self.mode_topic = rospy.get_param("~mode_topic", "/pi_mode")

        self.output_topic = rospy.get_param("~output_topic", "/yolo/annotated")
        self.output_topic_comp = rospy.get_param("~output_topic_comp", "/yolo/annotated/compressed")

        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))

        self.person_label = "person"
        self.timeout_yolo = float(rospy.get_param("~timeout_yolo", 0.5))
        self.timeout_cmd = float(rospy.get_param("~timeout_cmd", 1.0))

        # ---------- 내부 상태 ----------
        self.bridge = CvBridge()
        self.last_dets = []
        self.last_yolo_time = None

        self.v = 0.0
        self.w = 0.0
        self.state = 0
        self.mode_text = ""
        self.mode_time = None

        # ---------- 퍼블리셔 ----------
        self.pub_raw = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.pub_comp = rospy.Publisher(self.output_topic_comp, CompressedImage, queue_size=1)

        # ---------- 구독 ----------
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(self.yolo_topic, Yolo, self.yolo_callback, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber(self.state_topic, Int8, self.state_callback, queue_size=1)
        rospy.Subscriber(self.mode_topic, String, self.mode_callback, queue_size=1)

        rospy.loginfo("visualize_person_tracking started")
        rospy.loginfo("  image: %s", self.image_topic)
        rospy.loginfo("  yolo: %s", self.yolo_topic)
        rospy.loginfo("  cmd_vel: %s", self.cmd_topic)

    # ==========================================================
    # YOLO callback (사람만 필터링)
    # ==========================================================
    def yolo_callback(self, msg: Yolo):
        dets = []
        for d in msg.detections:
            label = (d.label or "").lower()
            if label == self.person_label:
                dets.append({
                    "xc": int(d.x_center),
                    "yc": int(d.y_center),
                    "w": int(d.w),
                    "h": int(d.h),
                    "label": d.label,
                    "conf": float(d.conf),
                })

        self.last_dets = dets
        self.last_yolo_time = rospy.Time.now()

    # ==========================================================
    # cmd_vel callback
    # ==========================================================
    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    # ==========================================================
    # state callback
    # ==========================================================
    def state_callback(self, msg: Int8):
        self.state = int(msg.data)

    # ==========================================================
    # mode text (finding person / person chase)
    # ==========================================================
    def mode_callback(self, msg: String):
        self.mode_text = str(msg.data)
        self.mode_time = rospy.Time.now()

    # ==========================================================
    # Main image callback
    # ==========================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        H, W = frame.shape[:2]
        cx, cy = W // 2, H // 2

        # ---------- 중앙 노란 십자 ----------
        cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # ---------- 모드 표시 (person chase 등) ----------
        mode_age = (rospy.Time.now() - self.mode_time).to_sec() if self.mode_time else 999
        if mode_age > self.timeout_cmd:
            mode_display = "finding person"
        else:
            mode_display = self.mode_text

        cv2.putText(frame, mode_display, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)

        # ---------- cmd_vel 표시 ----------
        cmd_text = f"v={self.v:.2f}, w={self.w:.2f}"
        cv2.putText(frame, cmd_text, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

        # ---------- YOLO 박스 (사람만) ----------
        fresh = (
            self.last_yolo_time is not None
            and (rospy.Time.now() - self.last_yolo_time).to_sec() < self.timeout_yolo
        )

        if fresh:
            for d in self.last_dets:
                x1 = int(d["xc"] - d["w"] / 2)
                y1 = int(d["yc"] - d["h"] / 2)
                x2 = int(d["xc"] + d["w"] / 2)
                y2 = int(d["yc"] + d["h"] / 2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
                label = f"{d['label']} {d['conf']:.2f}"
                cv2.putText(frame, label, (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
                cv2.circle(frame, (d["xc"], d["yc"]), 4, (0, 220, 0), -1)

        # ---------- 퍼블리시 ----------
        self.publish_frame(frame, msg.header)

    # ==========================================================
    # Publish raw + compressed
    # ==========================================================
    def publish_frame(self, frame, header):
        # raw
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header = header
            self.pub_raw.publish(img_msg)
        except:
            pass

        # compressed
        try:
            ok, enc = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                comp = CompressedImage()
                comp.header = header
                comp.format = "jpeg"
                comp.data = enc.tobytes()
                self.pub_comp.publish(comp)
        except:
            pass


if __name__ == "__main__":
    VisualizePersonTracking()
    rospy.spin()
