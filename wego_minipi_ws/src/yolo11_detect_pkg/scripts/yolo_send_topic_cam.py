#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo11_detect_pkg.msg import Yolo, YoloDetect
from rknnlite.api import RKNNLite

# ================== 경로/하이퍼파라미터 ==================
IMG_SIZE   = (640, 640)    # (W, H)
CONF_THRES = 0.25
IOU_THRES  = 0.45
DEFAULT_IMAGE_TOPIC = "/usb_cam/image_raw"
# ========================================================


# 비율 유지 리사이즈 + 패딩하여 new_shape(W,H)로 변환하고, 패딩/스케일 정보를 반환
def letterbox(bgr, new_shape=(640, 640), color=(114,114,114)):
    h0, w0 = bgr.shape[:2]
    r = min(new_shape[0] / w0, new_shape[1] / h0)
    w1, h1 = int(round(w0 * r)), int(round(h0 * r))
    resized = cv2.resize(bgr, (w1, h1), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_shape[1], new_shape[0], 3), color, dtype=resized.dtype)
    x0 = (new_shape[0] - w1) // 2
    y0 = (new_shape[1] - h1) // 2
    canvas[y0:y0+h1, x0:x0+w1] = resized
    return canvas, (x0, y0, r, (w0, h0))


# metadata.yaml에서 클래스 이름을 {id:name} dict로 로드
def load_class_names(path):
    try:
        import yaml
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        names = data.get("names", {})
        if isinstance(names, list): return {i: n for i, n in enumerate(names)}
        if isinstance(names, dict): return {int(k): v for k, v in names.items()}
    except Exception:
        pass
    return {}


class YoloInfo(object):
    # 노드 초기화, 퍼블리셔/서브스크라이버 설정, RKNN 로드
    def __init__(self):
        rospy.init_node("yolo_info_node", anonymous=False)

        rknn_path = rospy.get_param('~rknn_path', None)
        yaml_path = rospy.get_param('~yaml_path', None)

        self.image_topic = str(rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC))
        self.bridge = CvBridge()

        self.info_pub = rospy.Publisher("/YoloInfo", Yolo, queue_size=10)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.cam_cb, queue_size=1, buff_size=2**22
        )
        print(f"[INFO] Subscribing (Image): {self.image_topic}")

        self.class_names = load_class_names(yaml_path)

        self.rknn_available = False
        try:
            self.rknn = RKNNLite()
            if self.rknn.load_rknn(rknn_path) == 0 and \
               self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) == 0:
                self.rknn_available = True
                print("[INFO] RKNN ready.")
            else:
                print("[ERR ] RKNN load/init failed.")
        except Exception as e:
            print("[ERR ] RKNN init exception:", e)

        self._printed_shapes = False
        self._warned_shape = False

    # OpenCV BGR 프레임을 RKNN 입력 텐서(NHWC, uint8)와 메타 정보로 전처리
    def preprocess(self, bgr):
        lb, (pad_x, pad_y, scale, (w0, h0)) = letterbox(bgr, new_shape=IMG_SIZE)
        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
        nhwc = np.expand_dims(rgb.astype(np.uint8), axis=0)  # (1,H,W,3)
        meta = dict(pad_x=pad_x, pad_y=pad_y, scale=scale, w0=w0, h0=h0,
                    W=IMG_SIZE[0], H=IMG_SIZE[1])
        return nhwc, meta

    # 다양한 모델 출력 형태를 (N, C) 형태로 정규화
    def _as_NC(self, out0):
        a = np.array(out0)
        if a.ndim == 3:
            if a.shape[1] in (5, 6) or a.shape[1] > 6:
                a = np.transpose(a, (0, 2, 1))
            a = a.squeeze(0)
        elif a.ndim == 2:
            if a.shape[0] in (5, 6) or (a.shape[0] < a.shape[1] and a.shape[0] <= 8):
                a = a.T
        else:
            a = a.reshape(-1, a.shape[-1])
        return a

    # 간단 NMS (xywh dict 리스트 입력)
    @staticmethod
    def _nms_xywh(dets, iou_th=0.45):
        if not dets:
            return dets
        boxes = []
        for d in dets:
            xc, yc, w, h = d["x_center"], d["y_center"], d["w"], d["h"]
            x1 = xc - w/2.0; y1 = yc - h/2.0; x2 = xc + w/2.0; y2 = yc + h/2.0
            boxes.append([x1, y1, x2, y2, d["conf"], d["cls"], d])
        boxes.sort(key=lambda x: x[4], reverse=True)
        keep = []
        while boxes:
            a = boxes.pop(0)
            keep.append(a[-1])
            tmp = []
            for b in boxes:
                if a[5] != b[5]:
                    tmp.append(b); continue
                xx1=max(a[0],b[0]); yy1=max(a[1],b[1])
                xx2=min(a[2],b[2]); yy2=min(a[3],b[3])
                w=max(0.0,xx2-xx1); h=max(0.0,yy2-yy1)
                inter=w*h
                iou=inter/(((a[2]-a[0])*(a[3]-a[1]))+((b[2]-b[0])*(b[3]-b[1]))-inter+1e-6)
                if iou<=iou_th: tmp.append(b)
            boxes = tmp
        return keep

    # RKNN 출력 → 원본 이미지 좌표계의 검출 리스트(dict)로 변환
    def postprocess(self, outputs, meta):
        dets = []
        if not outputs:
            return dets

        arr = self._as_NC(outputs[0])
        if arr.ndim != 2 or arr.shape[1] < 5:
            if not self._warned_shape:
                print("[WARN] Unsupported output shape:", arr.shape)
                self._warned_shape = True
            return dets

        N, C = arr.shape
        W, H = meta["W"], meta["H"]
        pad_x, pad_y, scale = meta["pad_x"], meta["pad_y"], meta["scale"]
        w0, h0 = meta["w0"], meta["h0"]

        # 편의 함수: letterbox 좌표를 원본 좌표계로 되돌리기
        def to_orig_xywh(xc, yc, w, h):
            ox = (xc - pad_x) / scale
            oy = (yc - pad_y) / scale
            ow = w / scale
            oh = h / scale
            ox = max(0, min(w0 - 1, ox)); oy = max(0, min(h0 - 1, oy))
            ow = max(1, min(w0, ow));     oh = max(1, min(h0, oh))
            return int(ox), int(oy), int(ow), int(oh)

        a4 = arr[:, :4].astype(np.float32)
        is_norm = (a4.max() <= 1.5) and (a4.min() >= -0.2)

        # (xywh + conf) 형태
        if C == 5:
            xywh = a4.copy()
            if is_norm:
                xywh[:,0]*=W; xywh[:,1]*=H; xywh[:,2]*=W; xywh[:,3]*=H
            conf = arr[:, 4].astype(np.float32)

            cmax = float(conf.max()) if conf.size else 0.0
            if 1.5 < cmax <= 255.0: conf /= 255.0
            elif 255.0 < cmax <= 65535.0: conf /= 65535.0

            m = conf >= CONF_THRES
            xywh = xywh[m]; conf = conf[m]
            if xywh.size == 0: return dets

            x, y, w, h = xywh[:,0], xywh[:,1], xywh[:,2], xywh[:,3]
            boxes = np.stack([x-w/2, y-h/2, x+w/2, y+h/2], axis=1)
            boxes[:, [0,2]] = np.clip(boxes[:, [0,2]], 0, W-1)
            boxes[:, [1,3]] = np.clip(boxes[:, [1,3]], 0, H-1)
            order = conf.argsort()[::-1]

            keep = []
            while order.size > 0:
                i = order[0]
                keep.append(i)
                if order.size == 1: break
                rest = order[1:]
                xx1 = np.maximum(boxes[i,0], boxes[rest,0])
                yy1 = np.maximum(boxes[i,1], boxes[rest,1])
                xx2 = np.minimum(boxes[i,2], boxes[rest,2])
                yy2 = np.minimum(boxes[i,3], boxes[rest,3])
                w_ = np.maximum(0, xx2-xx1); h_ = np.maximum(0, yy2-yy1)
                inter = w_*h_
                area_i = (boxes[i,2]-boxes[i,0])*(boxes[i,3]-boxes[i,1])
                area_r = (boxes[rest,2]-boxes[rest,0])*(boxes[rest,3]-boxes[rest,1])
                iou = inter / (area_i + area_r - inter + 1e-6)
                order = rest[iou < IOU_THRES]

            xywh = xywh[keep]; conf = conf[keep]
            for (xc, yc, wv, hv), sc in zip(xywh, conf):
                ox, oy, ow, oh = to_orig_xywh(xc, yc, wv, hv)
                dets.append(dict(cls=0, conf=float(sc),
                                 x_center=ox, y_center=oy, w=ow, h=oh,
                                 size=int(ow*oh)))  # size 추가
            return dets

        # (xywh + obj + cls_probs) 형태
        if C > 6:
            xywh = a4
            if is_norm:
                xywh[:,0]*=W; xywh[:,1]*=H; xywh[:,2]*=W; xywh[:,3]*=H
            obj = arr[:,4:5].astype(np.float32)
            cls_probs = arr[:,5:].astype(np.float32)
            maxv = max(float(obj.max()), float(cls_probs.max()))
            if 1.5 < maxv <= 255.0: obj /= 255.0; cls_probs /= 255.0
            elif 255.0 < maxv <= 65535.0: obj /= 65535.0; cls_probs /= 65535.0

            cls_ids = np.argmax(cls_probs, axis=1)
            cls_conf = cls_probs[np.arange(N), cls_ids]
            scores = (obj.reshape(-1) * cls_conf.reshape(-1))
            for i in range(N):
                if scores[i] < CONF_THRES or xywh[i,2] <= 1 or xywh[i,3] <= 1: continue
                xc, yc, wv, hv = xywh[i]
                ox, oy, ow, oh = to_orig_xywh(xc, yc, wv, hv)
                dets.append(dict(cls=int(cls_ids[i]), conf=float(scores[i]),
                                 x_center=ox, y_center=oy, w=ow, h=oh))
            return self._nms_xywh(dets, IOU_THRES)

        # 기타 포맷(휴리스틱): (x1y1x2y2 + score + cls) 또는 (xywh + score + cls)
        x1y1x2y2_like = (np.mean(a4[:,2] - a4[:,0]) > 0) and (np.mean(a4[:,3] - a4[:,1]) > 0)
        col4_intlike = np.mean(np.abs(arr[:,4] - np.round(arr[:,4])) < 1e-3)
        col5_intlike = np.mean(np.abs(arr[:,5] - np.round(arr[:,5])) < 1e-3)
        if (col5_intlike > 0.8 and arr[:,5].max() < 1000):
            cls_col, score_col = 5, 4
        elif (col4_intlike > 0.8 and arr[:,4].max() < 1000):
            cls_col, score_col = 4, 5
        else:
            cls_col, score_col = 5, 4

        scores = arr[:, score_col].astype(np.float32)
        smax = float(scores.max()) if scores.size else 0.0
        if 1.5 < smax <= 255.0: scores /= 255.0
        elif 255.0 < smax <= 65535.0: scores /= 65535.0

        if x1y1x2y2_like:
            if is_norm:
                a4[:,0]*=W; a4[:,1]*=H; a4[:,2]*=W; a4[:,3]*=H
            x1,y1,x2,y2 = a4[:,0],a4[:,1],a4[:,2],a4[:,3]
            w = (x2-x1); h = (y2-y1); xc = x1 + w/2.0; yc = y1 + h/2.0
            cls_ids = arr[:, cls_col].astype(np.int32)
            for i in range(N):
                if scores[i] < CONF_THRES or w[i] <= 1 or h[i] <= 1: continue
                ox, oy, ow, oh = to_orig_xywh(xc[i], yc[i], w[i], h[i])
                dets.append(dict(cls=int(cls_ids[i]), conf=float(scores[i]),
                                 x_center=ox, y_center=oy, w=ow, h=oh))
            return self._nms_xywh(dets, IOU_THRES)
        else:
            xywh = a4
            if is_norm:
                xywh[:,0]*=W; xywh[:,1]*=H; xywh[:,2]*=W; xywh[:,3]*=H
            cls_ids = arr[:, cls_col].astype(np.int32)
            for i in range(N):
                if scores[i] < CONF_THRES or xywh[i,2] <= 1 or xywh[i,3] <= 1: continue
                xc, yc, wv, hv = xywh[i]
                ox, oy, ow, oh = to_orig_xywh(xc, yc, wv, hv)
                dets.append(dict(cls=int(cls_ids[i]), conf=float(scores[i]),
                                 x_center=ox, y_center=oy, w=ow, h=oh))
            return self._nms_xywh(dets, IOU_THRES)

    # ROS Image 콜백: 변환 → 추론/퍼블리시 (에러 원인 구분해서 로그)
    def cam_cb(self, msg: Image):
        # print("Image received. Starting conversion...")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"[ERR ] CvBridge conversion failed: {e}")
            return

        # print("Conversion completed. Starting inference...")
        if frame is None:
            print("[WARN] CvBridge returned None frame. Skipping inference.")
            return

        try:
            self._infer_and_publish(frame, header=msg.header)
        except Exception as e:
            print(f"[ERR ] Inference/publish failed: {e}")

    # 전처리 → RKNN 추론 → 후처리 → Yolo 메시지로 퍼블리시
    def _infer_and_publish(self, frame, header=None):
        inp, meta = self.preprocess(frame)
        dets_parsed = []
        if self.rknn_available:
            try:
                outputs = self.rknn.inference(inputs=[inp], data_format='nhwc')
                if not self._printed_shapes:
                    try:
                        print("[INFO] RKNN outputs:",
                              ", ".join([f"{i}:{o.shape},{o.dtype}" for i, o in enumerate(outputs)]))
                    except Exception:
                        pass
                    self._printed_shapes = True
                dets_parsed = self.postprocess(outputs, meta)
            except Exception as e:
                print("[ERR ] RKNN inference error:", e)
                dets_parsed = []

        out = Yolo()
        out.header = header if header is not None else Header(stamp=rospy.Time.now(), frame_id="camera")
        detections = []
        for d in dets_parsed:
            m = YoloDetect()
            m.class_id  = int(d["cls"])
            m.label     = self.class_names.get(m.class_id, "")
            m.x_center  = int(d["x_center"])
            m.y_center  = int(d["y_center"])
            m.w         = int(d["w"])
            m.h         = int(d["h"])
            # size 키가 없는 경로도 있으므로 안전하게 w*h로 대체
            m.size      = int(d.get("size", m.w * m.h))
            m.conf      = float(d["conf"])
            detections.append(m)

        out.detections = detections
        try:
            out.count = len(detections)
        except Exception:
            pass

        self.info_pub.publish(out)
        # print("YoloDetect message published.")

# 메인: 노드 실행
def main():
    YoloInfo()
    rospy.spin()

if __name__ == "__main__":
    main()
