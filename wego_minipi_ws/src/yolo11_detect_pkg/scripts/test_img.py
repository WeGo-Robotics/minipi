#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, argparse, yaml
import cv2
import numpy as np
from rknnlite.api import RKNNLite

# -------------------- 메타 로더 --------------------
def load_meta(meta_path):
    with open(meta_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    meta = {}
    # 입력 크기
    if "input_size" in raw:
        meta["input_size"] = list(raw["input_size"])
    elif "imgsz" in raw:
        isz = raw["imgsz"]
        if isinstance(isz, (list, tuple)) and len(isz) >= 2:
            meta["input_size"] = [int(isz[0]), int(isz[1])]
        else:
            meta["input_size"] = [640, 640]
    else:
        meta["input_size"] = [640, 640]

    # 클래스 이름
    if "names" in raw:
        if isinstance(raw["names"], dict):
            items = sorted(((int(k), v) for k, v in raw["names"].items()), key=lambda x: x[0])
            meta["class_names"] = [str(v) for _, v in items]
        else:
            meta["class_names"] = [str(x) for x in raw["names"]]
    else:
        meta["class_names"] = [str(i) for i in range(80)]

    meta["strides"]      = [8, 16, 32]  # raw-head용 기본값(이번 모델은 end2end라 사용 안됨)
    meta["conf_thres"]   = float(raw.get("conf_thres", 0.05))
    meta["iou_thres"]    = float(raw.get("iou_thres", 0.45))
    meta["nms_max_det"]  = int(raw.get("nms_max_det", 300))
    meta["rgb"]          = bool(raw.get("rgb", True))
    meta["letterbox"]    = bool(raw.get("letterbox", True))
    return meta

# -------------------- 전처리 --------------------
def letterbox(im, new_shape=(640,640), color=(114,114,114), scaleup=True):
    h0, w0 = im.shape[:2]
    r = min(new_shape[0]/w0, new_shape[1]/h0)
    if not scaleup: r = min(r, 1.0)
    new_unpad = (int(round(w0*r)), int(round(h0*r)))
    dw, dh = new_shape[0]-new_unpad[0], new_shape[1]-new_unpad[1]
    dw /= 2; dh /= 2
    if (w0, h0) != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh-0.1)), int(round(dh+0.1))
    left, right = int(round(dw-0.1)), int(round(dw+0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)

def scale_coords_xyxy(box, ratio, pad, orig_shape):
    x1,y1,x2,y2 = box
    dw, dh = pad
    x1 -= dw; x2 -= dw
    y1 -= dh; y2 -= dh
    x1 /= ratio; x2 /= ratio
    y1 /= ratio; y2 /= ratio
    h, w = orig_shape[:2]
    return [max(0, min(w-1, x1)),
            max(0, min(h-1, y1)),
            max(0, min(w-1, x2)),
            max(0, min(h-1, y2))]

# -------------------- 후처리 --------------------
def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

def process_end2end_output(out, conf_thres, class_names, lb_ratio, lb_pad, orig_shape, print_hist=True):
    """
    End2End RKNN 출력 자동 처리 (단일/다중 클래스 모두 지원)
    """
    o = out
    # (N,C)로 정규화
    if o.ndim == 3 and o.shape[1] in (5, 6, 7):
        o = o.transpose(0, 2, 1)[0]
    elif o.ndim == 3 and o.shape[2] in (5, 6, 7):
        o = o[0]
    elif o.ndim == 2 and o.shape[0] in (5, 6, 7):
        o = o.transpose(1, 0)
    elif o.ndim == 3 and o.shape[0] == 1:
        o = o[0]
    o = o.astype(np.float32)

    if not (o.ndim == 2 and o.shape[1] in (5, 6, 7)):
        raise ValueError(f"Unsupported end2end output shape: {out.shape}")

    C = o.shape[1]
    xyxy = o[:, :4].copy()
    H, W = orig_shape[:2]

    # 좌표 스케일 복원
    if np.max(xyxy) <= 2.0:
        xyxy[:, [0, 2]] *= W
        xyxy[:, [1, 3]] *= H

    tail_idxs = list(range(4, C))
    cand = o[:, tail_idxs]

    # ---- 단일 클래스 전용 모드 ----
    single_class = len(class_names) == 1
    cls_col_idx = None
    score_col_idx = None

    if single_class:
        # 단일 클래스 모델은 cls=0으로 고정
        cls_col_idx = None
        score_col_idx = 4 if C >= 5 else None
    else:
        # 기존 방식 유지
        def looks_like_cls(col):
            v = col
            if v.size == 0:
                return False
            vmax, vmin = float(np.max(v)), float(np.min(v))
            intish = np.mean(np.abs(v - np.round(v)) < 0.1) > 0.8
            within = (vmin >= -0.5) and (vmax <= (len(class_names) - 0.5))
            return intish and within

        for j in range(cand.shape[1]):
            if looks_like_cls(cand[:, j]):
                cls_col_idx = tail_idxs[j]
                break

        # 점수열 추정
        rest = [k for k in tail_idxs if k != cls_col_idx] if cls_col_idx is not None else tail_idxs[:]
        def score_pref(col):
            v = col
            frac01 = np.mean((v >= 0.0) & (v <= 1.0))
            spread = np.max(np.abs(v))
            return frac01, -spread

        best = None
        for k in rest:
            s = score_pref(o[:, k])
            if (best is None) or (s > best[0]):
                best = (s, k)
        score_col_idx = best[1] if best else rest[0]

        if cls_col_idx is None:
            alt = [k for k in tail_idxs if k != score_col_idx]
            cls_col_idx = alt[0] if alt else 5 if C >= 6 else tail_idxs[-1]

    # ---- 값 추출 ----
    raw_scores = o[:, score_col_idx] if score_col_idx is not None else np.ones(o.shape[0])
    cls_ids = np.zeros(o.shape[0], dtype=int) if cls_col_idx is None else np.round(o[:, cls_col_idx]).astype(int)

    # 점수 정규화 (logit→sigmoid)
    if (np.mean((raw_scores >= 0.0) & (raw_scores <= 1.0)) < 0.2) or (np.max(np.abs(raw_scores)) > 5.0):
        raw_scores = 1.0 / (1.0 + np.exp(-raw_scores))

    dets_all = []
    for (x1, y1, x2, y2), sc, cid in zip(xyxy, raw_scores, cls_ids):
        x1, y1, x2, y2 = scale_coords_xyxy([x1, y1, x2, y2], lb_ratio, lb_pad, orig_shape)
        dets_all.append([x1, y1, x2, y2, float(sc), int(cid)])

    # 디버그: 클래스 히스토그램
    if print_hist and len(dets_all):
        arr = np.array(dets_all)
        arr = arr[np.argsort(-arr[:, 4])][:200]
        u, c = np.unique(arr[:, 5].astype(int), return_counts=True)
        print("hist (top200 by score):", dict(zip(u.tolist(), c.tolist())))

    # ---- 클래스별 임계치 ----
    per_cls_th = {
        0: max(conf_thres, 0.25),
        1: max(conf_thres, 0.10),
        2: max(conf_thres, 0.10),
    }

    dets = []
    for x1, y1, x2, y2, sc, cid in dets_all:
        if cid not in per_cls_th:
            cid = 0 if single_class else cid
        th = per_cls_th.get(int(cid), conf_thres)
        if sc >= th:
            dets.append([x1, y1, x2, y2, sc, int(cid)])

    return dets, dets_all

# -------------------- 시각화 --------------------
def draw_dets(img, dets, class_names, color=(0,255,0), thickness=2):
    for x1,y1,x2,y2,score,cid in dets:
        p1, p2 = (int(x1),int(y1)), (int(x2),int(y2))
        cv2.rectangle(img,p1,p2,color,thickness)
        label = f"{class_names[cid] if 0 <= cid < len(class_names) else cid} {score:.2f}"
        t_sz,_ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img,(p1[0],p1[1]-t_sz[1]-6),(p1[0]+t_sz[0]+4,p1[1]),color,-1)
        cv2.putText(img,label,(p1[0]+2,p1[1]-4),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
    return img

# -------------------- 메인 --------------------
def run_inference(rknn_path, image_path, meta_path, out_path, core_mask=None,
                  conf_override=None, debug_low=False, low_th=0.005):
    meta = load_meta(meta_path)
    if conf_override is not None:
        meta["conf_thres"] = float(conf_override)

    in_w, in_h = meta["input_size"]
    use_lb     = meta["letterbox"]
    to_rgb     = True  # Ultralytics 기본 RGB

    img0 = cv2.imread(image_path)
    if img0 is None:
        raise FileNotFoundError(image_path)

    if use_lb:
        img, ratio, pad = letterbox(img0.copy(), (in_w,in_h))
    else:
        img = cv2.resize(img0.copy(), (in_w,in_h))
        ratio, pad = min(in_w/img0.shape[1], in_h/img0.shape[0]), (0,0)

    img_in = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if to_rgb else img
    img_in = img_in.astype(np.uint8)

    rknn = RKNNLite()
    if rknn.load_rknn(rknn_path) != 0:
        raise RuntimeError("RKNN load failed")
    if core_mask is not None:
        rknn.init_runtime(core_mask=core_mask)
    else:
        rknn.init_runtime()

    # NCHW 입력
    inputs = [img_in.transpose(2,0,1)[None,...]]
    outputs = rknn.inference(inputs=inputs)
    for i,o in enumerate(outputs):
        print(f"head{i}: shape={o.shape}, dtype={o.dtype}, min={o.min():.3f}, max={o.max():.3f}")
    rknn.release()

    # end2end 처리
    dets, dets_all = [], []
    if len(outputs)==1 and outputs[0].ndim>=2 and outputs[0].shape[-1]>=6:
        dets, dets_all = process_end2end_output(outputs[0], meta["conf_thres"], meta["class_names"],
                                                ratio, pad, img0.shape, print_hist=True)
    else:
        print("[WARN] unexpected output structure; no detections decoded")

    # 메인 결과
    out_img = draw_dets(img0.copy(), dets, meta["class_names"], color=(0,255,0), thickness=2)
    cv2.imwrite(out_path, out_img)

    # 저신뢰도 디버그 시각화
    if debug_low and len(dets_all):
        low = [d for d in dets_all if (d[4] < meta["conf_thres"] and d[4] >= low_th)]
        if low:
            img_dbg = draw_dets(img0.copy(), low, meta["class_names"], color=(0,255,255), thickness=1)
            cv2.imwrite("detect_low.jpg", img_dbg)
            print(f"[DEBUG] low-score boxes visualized to detect_low.jpg (>= {low_th})")

    return dets, out_path

# -------------------- CLI --------------------
def parse_args():
    ap = argparse.ArgumentParser(description="YOLO11 RKNNLite End2End detector")
    ap.add_argument("--rknn", required=True, help="path to model.rknn")
    ap.add_argument("--image", required=True, help="path to input image")
    ap.add_argument("--meta", required=True, help="path to metadata.yaml")
    ap.add_argument("--out", default="detect.jpg", help="output image path")
    ap.add_argument("--conf", type=float, default=None, help="override conf threshold (e.g., 0.05)")
    ap.add_argument("--debug-low", action="store_true", help="draw low-score boxes to detect_low.jpg")
    ap.add_argument("--low-th", type=float, default=0.005, help="low-score draw threshold")
    ap.add_argument("--all-cores", action="store_true", help="use all NPU cores if available")
    return ap.parse_args()

if __name__ == "__main__":
    args = parse_args()
    core_mask = None
    if getattr(args, "all_cores", False) and hasattr(RKNNLite, "NPU_CORE_0_1_2"):
        core_mask = RKNNLite.NPU_CORE_0_1_2

    dets, saved = run_inference(
        args.rknn, args.image, args.meta, args.out,
        core_mask=core_mask, conf_override=args.conf,
        debug_low=args.debug_low, low_th=args.low_th
    )
    print(f"[OK] saved: {saved}")
    if not dets:
        print("No detections.")
    else:
        for x1,y1,x2,y2,conf,cid in dets:
            print(f"cls={cid} conf={conf:.3f} box=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f})")


# python3 test_img.py \
#   --rknn /home/hightorque/soccer_ws/src/yolo11_detect_pkg/config/rknn/roboworld_best2-rk3588.rknn \
#   --image /home/hightorque/soccer_ws/src/yolo11_detect_pkg/scripts/test.jpg \
#   --meta  /home/hightorque/soccer_ws/src/yolo11_detect_pkg/config/rknn/roboworld_metadata2.yaml \
#   --out   detect.jpg \

