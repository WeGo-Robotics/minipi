#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import threading

import rospy
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np

# AnyMsg -> 실제 메시지 복원
from roslib.message import get_message_class


def deserialize_any(any_msg):
    """rospy.AnyMsg -> (mtype:str, msg_obj or None)"""
    try:
        mtype = any_msg._connection_header.get('type', '')
    except Exception:
        return "", None
    klass = get_message_class(mtype)
    if klass is None:
        return mtype, None
    obj = klass()
    try:
        obj.deserialize(any_msg._buff)
    except Exception:
        return mtype, None
    return mtype, obj


class YoloVisualizer(object):
    def __init__(self):
        # ---- Params ----
        self.image_topic   = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.info_topic    = rospy.get_param("~info_topic",  "/YoloInfo")
        self.use_compressed = bool(rospy.get_param("~use_compressed", False))
        self.window_name   = rospy.get_param("~window_name", "YOLO Visualizer")
        self.font_scale    = float(rospy.get_param("~font_scale", 0.6))
        self.thickness     = int(rospy.get_param("~thickness", 2))
        self.conf_threshold= float(rospy.get_param("~conf_threshold", 0.0))
        self.draw_score    = bool(rospy.get_param("~draw_score", True))
        self.draw_label    = bool(rospy.get_param("~draw_label", True))
        self.publish_image = bool(rospy.get_param("~publish_image", True))
        self.pub_topic     = rospy.get_param("~pub_topic", "/yolo/visualized")
        self.bgr_box       = tuple(int(x) for x in rospy.get_param("~box_color_bgr", [0, 255, 0]))  # [B,G,R]

        # ---- State ----
        self.bridge = CvBridge()
        self.frame_bgr = None
        self.frame_lock = threading.Lock()
        self.last_boxes = []  # list of (x1,y1,x2,y2,label,conf)
        self.last_boxes_stamp = 0.0

        # ---- ROS IO ----
        if self.use_compressed:
            rospy.Subscriber(self.image_topic + "/compressed", RosCompressedImage, self.cb_image_compressed, queue_size=1)
        else:
            rospy.Subscriber(self.image_topic, RosImage, self.cb_image, queue_size=1)

        rospy.Subscriber(self.info_topic, rospy.AnyMsg, self.cb_yolo_any, queue_size=5)

        self.pub_vis = rospy.Publisher(self.pub_topic, RosImage, queue_size=1) if self.publish_image else None

        # ---- OpenCV Window ----
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)

        rospy.loginfo("[yolo_visualizer] image_topic=%s  info_topic=%s  compressed=%s",
                      self.image_topic, self.info_topic, self.use_compressed)

    # ----------------- Callbacks -----------------
    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cv_bridge img err: %s", e)
            return
        with self.frame_lock:
            self.frame_bgr = frame

    def cb_image_compressed(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "compressed decode err: %s", e)
            return
        if frame is None:
            return
        with self.frame_lock:
            self.frame_bgr = frame

    def cb_yolo_any(self, any_msg):
        mtype, real = deserialize_any(any_msg)
        boxes = []
        try:
            # /YoloInfo 패턴: msg.detections[*] with x_center,y_center,w,h,label,conf
            if real is not None and hasattr(real, "detections"):
                for d in real.detections:
                    # center-based
                    if all(hasattr(d, k) for k in ["x_center", "y_center", "w", "h"]):
                        cx, cy, w, h = int(d.x_center), int(d.y_center), int(d.w), int(d.h)
                        x1, y1 = int(cx - w/2), int(cy - h/2)
                        x2, y2 = int(cx + w/2), int(cy + h/2)
                    # or corner-based
                    elif all(hasattr(d, k) for k in ["xmin", "ymin", "xmax", "ymax"]):
                        x1, y1, x2, y2 = int(d.xmin), int(d.ymin), int(d.xmax), int(d.ymax)
                    else:
                        continue
                    lab = getattr(d, "label", "")
                    print(f"label: {lab}")
                    cnf = getattr(d, "conf", None)
                    cnf = None if cnf is None else float(cnf)
                    if self.conf_threshold > 0 and (cnf is not None) and (cnf < self.conf_threshold):
                        continue
                    boxes.append((x1, y1, x2, y2, str(lab), cnf))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[parse YoloInfo] %s: %s", mtype, e)
            boxes = []

        self.last_boxes = boxes
        self.last_boxes_stamp = time.time()

    # ----------------- Draw -----------------
    def draw(self, frame):
        """Return overlayed frame."""
        if frame is None:
            return None
        over = frame.copy()
        for (x1, y1, x2, y2, label, conf) in self.last_boxes:
            # clamp
            h, w = over.shape[:2]
            x1c = max(0, min(w-1, x1)); x2c = max(0, min(w-1, x2))
            y1c = max(0, min(h-1, y1)); y2c = max(0, min(h-1, y2))

            # box
            cv2.rectangle(over, (x1c, y1c), (x2c, y2c), self.bgr_box, self.thickness)

            # tag
            parts = []
            if self.draw_label and label:
                parts.append(label)
            if self.draw_score and conf is not None:
                parts.append(f"{conf:.2f}")
            tag = " ".join(parts) if parts else ""
            if tag:
                (tw, th), bl = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, 1)
                y_text = max(0, y1c - 4)
                cv2.rectangle(over, (x1c, y_text - th - bl), (x1c + tw + 6, y_text), self.bgr_box, -1)
                cv2.putText(over, tag, (x1c + 3, y_text - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 0, 0), 1, cv2.LINE_AA)
        return over

    # ----------------- Main Loop -----------------
    def spin(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame = None if self.frame_bgr is None else self.frame_bgr.copy()
            if frame is not None:
                vis = self.draw(frame)
                if vis is not None:
                    cv2.imshow(self.window_name, vis)
                    cv2.waitKey(1)
                    if self.pub_vis is not None:
                        try:
                            msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                            self.pub_vis.publish(msg)
                        except Exception as e:
                            rospy.logwarn_throttle(2.0, "publish err: %s", e)
            rate.sleep()

    def __del__(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main():
    rospy.init_node("yolo_visualizer", anonymous=True)
    node = YoloVisualizer()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
