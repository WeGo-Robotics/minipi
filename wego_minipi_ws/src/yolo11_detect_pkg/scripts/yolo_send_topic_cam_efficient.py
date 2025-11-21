#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
from std_msgs.msg import Header, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo11_detect_pkg.msg import Yolo, YoloDetect
from rknnlite.api import RKNNLite

# ================== 경로/하이퍼파라미터 ==================
IMG_SIZE = (640, 640)  # (W, H)
CONF_THRES = 0.8
IOU_THRES = 0.45
DEFAULT_IMAGE_TOPIC = "/usb_cam/image_raw"
# DEFAULT_IMAGE_TOPIC = "/camera/color/image_raw"
# YOLO 제어를 위한 상수
YOLO_CONTROL_TOPIC = "/yolo_run_control"
YOLO_CONTROL_RUN, YOLO_CONTROL_STOP = 1, 0  # 1: Run, 0: Stop
# ========================================================


# 비율 유지 리사이즈 + 패딩하여 new_shape(W,H)로 변환하고, 패딩/스케일 정보를 반환
def letterbox(bgr, new_shape=(640, 640), color=(114, 114, 114)):
    h0, w0 = bgr.shape[:2]
    r = min(new_shape[0] / w0, new_shape[1] / h0)
    w1, h1 = int(round(w0 * r)), int(round(h0 * r))
    resized = cv2.resize(bgr, (w1, h1), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_shape[1], new_shape[0], 3), color, dtype=resized.dtype)
    x0 = (new_shape[0] - w1) // 2
    y0 = (new_shape[1] - h1) // 2
    canvas[y0 : y0 + h1, x0 : x0 + w1] = resized
    return canvas, (x0, y0, r, (w0, h0))


# metadata.yaml에서 클래스 이름을 {id:name} dict로 로드
def load_class_names(path):
    try:
        import yaml

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        names = data.get("names", {})
        if isinstance(names, list):
            return {i: n for i, n in enumerate(names)}
        if isinstance(names, dict):
            return {int(k): v for k, v in names.items()}
    except Exception:
        pass
    return {}


class YoloInfo(object):
    # 노드 초기화, 퍼블리셔/서브스크라이버 설정, RKNN 로드
    def __init__(self):
        rospy.init_node("yolo_info_node", anonymous=False)

        rknn_path = rospy.get_param("~rknn_path", None)
        yaml_path = rospy.get_param("~yaml_path", None)
        self.yolo_is_running = True

        self.image_topic = str(rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC))
        self.bridge = CvBridge()

        self.info_pub = rospy.Publisher("/YoloInfo", Yolo, queue_size=10)

        # YOLO 제어를 위한 토픽 구독
        self.control_sub = rospy.Subscriber(YOLO_CONTROL_TOPIC, Int8, self.control_cb, queue_size=1)
        print(f"[INFO] Subscribing (Control): {YOLO_CONTROL_TOPIC}")

        # 기존 이미지 서브스크라이버
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cam_cb, queue_size=1, buff_size=2**22)
        print(f"[INFO] Subscribing (Image): {self.image_topic}")

        self.class_names = load_class_names(yaml_path)
        self.num_classes = len(self.class_names) # 클래스 개수 저장 (C=7 대응에 사용)
        print(f"[INFO] Loaded class names from {yaml_path}:")
        for k, v in self.class_names.items():
            print(f"  ID {k}: '{v}'")
        print(f"[INFO] Total number of classes: {self.num_classes}")


        self.rknn_available = False
        try:
            self.rknn = RKNNLite()
            # NPU_CORE_0_1_2는 RK3588 계열에서 사용 가능
            if self.rknn.load_rknn(rknn_path) == 0 and self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) == 0:
                self.rknn_available = True
                print("[INFO] RKNN ready.")
            else:
                print("[ERR ] RKNN load/init failed.")
        except Exception as e:
            print("[ERR ] RKNN init exception:", e)

        self._printed_shapes = False
        self._warned_shape = False

        # YOLO 실행 상태 플래그 (기본값: 실행)
        self.yolo_is_running = True

    # YOLO 제어 콜백 함수
    def control_cb(self, msg: Int8):
        """YOLO Run/Stop 신호를 받아 상태를 업데이트합니다."""
        desired_state = msg.data == YOLO_CONTROL_RUN
        if self.yolo_is_running != desired_state:
            self.yolo_is_running = desired_state
            print(f"[INFO] YOLO processing {'ENABLED' if self.yolo_is_running else 'DISABLED'} by external control.")

    # OpenCV BGR 프레임을 RKNN 입력 텐서(NHWC, uint8)와 메타 정보로 전처리
    def preprocess(self, bgr):
        lb, (pad_x, pad_y, scale, (w0, h0)) = letterbox(bgr, new_shape=IMG_SIZE)
        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
        nhwc = np.expand_dims(rgb.astype(np.uint8), axis=0)  # (1,H,W,3)
        meta = dict(pad_x=pad_x, pad_y=pad_y, scale=scale, w0=w0, h0=h0, W=IMG_SIZE[0], H=IMG_SIZE[1])
        return nhwc, meta

    # 다양한 모델 출력 형태를 (N, C) 형태로 정규화
    def _as_NC(self, out0):
        a = np.array(out0)
        # N이 8400 (proposals), C가 5 or 7 or 8 (channels)일 때
        # YOLOv8은 (1, C, N), YOLOv5는 (1, N, C) 형태로 나옴
        
        # 텐서의 두 번째 차원이 클래스 관련 차원일 경우 (C < N)
        C_candidate = a.shape[1] if a.ndim >= 2 else 0
        
        # (1, C, N) 형태일 경우 (C < N)
        if a.ndim == 3 and C_candidate < a.shape[2] and C_candidate >= 5:
            # (1, C, N) -> (1, N, C)로 변환
            a = np.transpose(a, (0, 2, 1)) 
        
        if a.ndim > 1:
            a = a.reshape(-1, a.shape[-1]) # (N, C)
        
        return a

    # 간단 NMS (xywh dict 리스트 입력)
    @staticmethod
    def _nms_xywh(dets, iou_th=0.45):
        if not dets:
            return dets
        boxes = []
        for d in dets:
            # 원본 좌표계의 xywh
            xc, yc, w, h = d["x_center"], d["y_center"], d["w"], d["h"]
            x1 = xc - w / 2.0
            y1 = yc - h / 2.0
            x2 = xc + w / 2.0
            y2 = yc + h / 2.0
            boxes.append([x1, y1, x2, y2, d["conf"], d["cls"], d])
        
        # Score 기준으로 내림차순 정렬
        boxes.sort(key=lambda x: x[4], reverse=True)
        
        keep = []
        while boxes:
            a = boxes.pop(0)
            keep.append(a[-1])
            tmp = []
            for b in boxes:
                # 클래스가 다르면 NMS 적용하지 않음 (다중 클래스 NMS)
                if a[5] != b[5]:
                    tmp.append(b)
                    continue
                
                # IOU 계산
                xx1 = max(a[0], b[0])
                yy1 = max(a[1], b[1])
                xx2 = min(a[2], b[2])
                yy2 = min(a[3], b[3])
                w = max(0.0, xx2 - xx1)
                h = max(0.0, yy2 - yy1)
                inter = w * h
                iou = inter / (((a[2] - a[0]) * (a[3] - a[1])) + ((b[2] - b[0]) * (b[3] - b[1])) - inter + 1e-6)
                
                if iou <= iou_th:
                    tmp.append(b)
            boxes = tmp
        return keep

    # RKNN 출력 → 원본 이미지 좌표계의 검출 리스트(dict)로 변환
    def postprocess(self, outputs, meta):
        dets = []
        if not outputs or self.num_classes == 0:
            return dets

        arr = self._as_NC(outputs[0])
        if arr.ndim != 2 or arr.shape[1] < 5:
            if not self._warned_shape:
                print(f"[WARN] Unsupported output shape: {arr.shape} (Expected 2D array with C>=5)")
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
            ox = max(0, min(w0 - 1, ox))
            oy = max(0, min(h0 - 1, oy))
            ow = max(1, min(w0, ow))
            oh = max(1, min(h0, oh))
            return int(ox), int(oy), int(ow), int(oh)

        # 1. 박스 좌표 (xywh) 추출 및 정규화 해제
        a4 = arr[:, :4].astype(np.float32)
        # 출력이 0~1 사이로 정규화되었는지 확인
        is_norm = (a4.max() <= 1.5) and (a4.min() >= -0.2)
        if is_norm:
            a4[:, 0] *= W
            a4[:, 1] *= H
            a4[:, 2] *= W
            a4[:, 3] *= H
        xywh = a4

        # 2. 채널 구조 파악: obj_conf 유무에 따른 동적 인덱스 설정
        # 표준 YOLO 출력: 4 (Box) + 1 (Obj Conf) + N_cls (Class Probs)
        C_FULL = 4 + 1 + self.num_classes 
        # Obj Conf 누락 출력: 4 (Box) + N_cls (Class Probs)
        C_TRUNCATED = 4 + self.num_classes 

        has_obj_conf = False
        obj_conf_idx = -1
        cls_probs_start_idx = -1
        
        if C == C_FULL:
            # Case 1: Obj Conf 존재 (예: 3클래스 C=8, 1클래스 C=6)
            has_obj_conf = True
            obj_conf_idx = 4
            cls_probs_start_idx = 5
        elif C == C_TRUNCATED:
            # Case 2: Obj Conf 누락 (예: 3클래스 C=7, 1클래스 C=5)
            has_obj_conf = False
            cls_probs_start_idx = 4
        else:
            # Case 3: 예상치 못한 채널 수 (오류 또는 휴리스틱 로직 사용)
            # 여기서는 기존 휴리스틱 로직을 간소화하여 오류로 처리하고 로깅.
            print(f"[ERROR] Unexpected output channel count C={C}. Expected {C_FULL} or {C_TRUNCATED} for {self.num_classes} classes.")
            return dets

        # 3. 점수 및 클래스 추출 (통합 로직)
        
        # Obj Conf 추출 및 정규화
        obj_conf = None
        if has_obj_conf:
            obj_conf = arr[:, obj_conf_idx].astype(np.float32)
            max_obj_conf = float(obj_conf.max())
            if max_obj_conf > 1.5:
                # 255/65535 등으로 정규화되지 않은 경우, 최대값으로 정규화
                obj_conf /= max_obj_conf
        
        # Class Probs 추출 및 정규화
        cls_probs = arr[:, cls_probs_start_idx:].astype(np.float32)
        max_cls_probs = float(cls_probs.max())
        if max_cls_probs > 1.5:
             # 255/65535 등으로 정규화되지 않은 경우, 최대값으로 정규화
            cls_probs /= max_cls_probs

        # 최대 클래스 ID 및 확률 결정
        cls_ids = np.argmax(cls_probs, axis=1)
        max_cls_conf = cls_probs[np.arange(N), cls_ids] 

        # 최종 점수 계산
        if has_obj_conf:
            # Obj Conf가 있는 경우: Obj Conf * Max Class Conf
            scores = obj_conf.reshape(-1) * max_cls_conf.reshape(-1)
            print(f"[INFO] Using Obj Conf * Max Class Conf for scoring (C={C_FULL}).")
        else:
            # Obj Conf가 없는 경우: Max Class Conf를 최종 점수로 사용 (C=7 해결책)
            scores = max_cls_conf.reshape(-1)
            print(f"[INFO] Using Max Class Conf only for scoring (C={C_TRUNCATED}).")


        # 4. 필터링 및 디테일 구성
        for i in range(N):
            score = scores[i]
            if score < CONF_THRES or xywh[i, 2] <= 1 or xywh[i, 3] <= 1:
                continue
            
            xc, yc, wv, hv = xywh[i]
            ox, oy, ow, oh = to_orig_xywh(xc, yc, wv, hv)
            
            # 단일 클래스 모델(num_classes=1)이고 obj_conf가 없는 C=5 케이스를 위해 cls=0으로 고정
            current_cls_id = int(cls_ids[i]) if self.num_classes > 1 else 0

            dets.append(dict(
                cls=current_cls_id, 
                conf=float(score), 
                x_center=ox, 
                y_center=oy, 
                w=ow, 
                h=oh, 
                size=ow * oh
            ))
        
        # NMS 적용 후 반환
        return self._nms_xywh(dets, IOU_THRES)

    # ROS Image 콜백: 변환 → 추론/퍼블리시 (에러 원인 구분해서 로그)
    def cam_cb(self, msg: Image):
        # YOLO가 정지 상태일 경우 즉시 리턴하여 추론을 건너뜀
        if not self.yolo_is_running:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"[ERR ] CvBridge conversion failed: {e}")
            return

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
                # 데이터 포맷을 nhwc로 명시
                outputs = self.rknn.inference(inputs=[inp], data_format="nhwc")
                if not self._printed_shapes:
                    try:
                        print("[INFO] RKNN outputs:", ", ".join([f"{i}:{o.shape},{o.dtype}" for i, o in enumerate(outputs)]))
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
            m.class_id = int(d["cls"])
            m.label = self.class_names.get(m.class_id, "")
            m.x_center = int(d["x_center"])
            m.y_center = int(d["y_center"])
            m.w = int(d["w"])
            m.h = int(d["h"])
            m.size = int(d.get("size", m.w * m.h))
            m.conf = float(d["conf"])
            detections.append(m)

        out.detections = detections
        try:
            out.count = len(detections)
        except Exception:
            pass

        self.info_pub.publish(out)


# 메인: 노드 실행
def main():
    YoloInfo()
    rospy.spin()


if __name__ == "__main__":
    main()
