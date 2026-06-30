#!/usr/bin/env python3
"""Cart detect-server for the Booster K1 seek-cart stack.

Open-vocabulary cart detector (YOLO-World, zero-shot — no training). Mirrors
receipt_validator/detect_server.py but detects the utility cart instead of a barcode.
Pulls the head-cam feed (``:8080/frame.jpg``), runs YOLO-World with the text prompt
"cart", applies a temporal stability gate, and serves the boxes over HTTP so the
torch-free ROS node ``cart_locate`` can read them:

  GET /detections.json  -> [{"kind":"cart","bbox":[x1,y1,x2,y2],"score":s,"stable":bool}, ...]
  GET /frame.jpg        -> the latest frame, annotated
  GET /status           -> fps + detection count

Torch lives HERE (run in the navila conda env: ultralytics YOLO-World + CUDA). rclpy
lives in system python 3.10 -> they don't share an interpreter, so the detector is an
HTTP service, not a ROS node (same split as the barcode stack).

Run:
  ~/miniconda3/envs/navila/bin/python cart_detect_server.py \
      --src http://192.168.10.102:8080/frame.jpg --port 8090
Offline test (loop a still image as the "feed"):
  ... --src file:///home/boosterk1/Desktop/Cart.jpeg
"""
import argparse
import json
import threading
import time
import urllib.request

import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLOWorld

PROMPTS = ["cart", "utility cart", "service cart"]


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = b""
        self.dets = []
        self.fps = 0.0

    def set(self, jpeg, dets, fps):
        with self.lock:
            self.jpeg, self.dets, self.fps = jpeg, dets, fps

    def snap(self):
        with self.lock:
            return self.jpeg, list(self.dets), self.fps


STATE = State()


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def _read_frame(src, opener):
    """Return a BGR frame from an http(s) :8080/frame.jpg or a file:// still."""
    if src.startswith("file://"):
        return cv2.imread(src[7:], cv2.IMREAD_COLOR)
    buf = opener.open(src, timeout=3).read()
    return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)


def grab_loop(src, model, conf, gate_k, gate_n, period):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    history = []          # last gate_n frames' raw boxes, for the temporal gate
    while True:
        t0 = time.time()
        try:
            frame = _read_frame(src, opener)
        except Exception:
            time.sleep(period)
            continue
        if frame is None:
            time.sleep(period)
            continue
        res = model.predict(frame, conf=conf, verbose=False)[0]
        boxes = []
        for b in res.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            boxes.append({"kind": "cart", "bbox": [x1, y1, x2, y2],
                          "score": float(b.conf[0])})
        boxes.sort(key=lambda d: d["score"], reverse=True)
        history.append([d["bbox"] for d in boxes])
        del history[:-gate_n]
        for d in boxes:                              # stable = matched in >=k of last n frames
            cnt = sum(1 for fb in history if any(_iou(d["bbox"], o) > 0.4 for o in fb))
            d["stable"] = cnt >= gate_k
        # annotate
        vis = frame.copy()
        for d in boxes:
            x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
            col = (0, 200, 0) if d["stable"] else (0, 160, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), col, 3)
            cv2.putText(vis, f"cart {d['score']:.2f}{'*' if d['stable'] else ''}",
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        ok, enc = cv2.imencode(".jpg", vis)
        fps = 1.0 / max(time.time() - t0, 1e-3)
        STATE.set(enc.tobytes() if ok else b"", boxes, fps)
        dt = period - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        jpeg, dets, fps = STATE.snap()
        if self.path.startswith("/frame.jpg"):
            self._send(jpeg, "image/jpeg") if jpeg else self._send(b"no frame yet", "text/plain", 503)
        elif self.path.startswith("/detections.json"):
            self._send(json.dumps(dets).encode(), "application/json")
        else:
            s = sum(1 for d in dets if d.get("stable"))
            self._send(f"cart detect-server\nfps: {fps:.1f}\ndetections: {len(dets)} ({s} stable)\n"
                       .encode(), "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="http://192.168.10.102:8080/frame.jpg")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--model", default="/home/boosterk1/models/yolov8x-world.pt",
                    help="yolov8x-world (big) resolves the distant/dark cart far better than yolov8s "
                         "(0.28 vs 0.05 on a live lab cart); pass yolov8s-world.pt only if x is too slow")
    ap.add_argument("--conf", type=float, default=0.05,
                    help="low YOLO-World threshold; the temporal gate filters noise")
    ap.add_argument("--k", type=int, default=3, help="stable after k of n frames")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--hz", type=float, default=4.0)
    a = ap.parse_args()
    model = YOLOWorld(a.model)
    model.set_classes(PROMPTS)
    threading.Thread(target=grab_loop,
                     args=(a.src, model, a.conf, a.k, a.n, 1.0 / a.hz), daemon=True).start()
    httpd = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    print(f"[cart-detect] http://0.0.0.0:{a.port}/detections.json  "
          f"src={a.src} model={a.model} prompts={PROMPTS}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
