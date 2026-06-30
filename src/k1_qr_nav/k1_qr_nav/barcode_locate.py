#!/usr/bin/env python3
"""Locate the barcode the robot can see, and publish it for Nav2 to walk to.

The heavy detector (YOLO/torch) stays in the project's `detect_server.py` (run in the
receipt_validator .venv): it grabs the robot head-cam feed (:8080/frame.jpg), runs the
barcode detector + temporal gate, and serves the boxes at :8090/detections.json. THIS
node is torch-free (plain rclpy on system python): it polls that JSON, takes the best
*stable* barcode box, projects its pixel centre through the camera intrinsics into the
`head_camera_optical` frame, and publishes:

  /qr_detection   (geometry_msgs/PointStamped)  -> qr_goal turns it into a standoff goal
  /barcode_bearing(std_msgs/Float64MultiArray [u_norm, score]) -> head_search centres on it

Range: if a metric depth is wired (assumed_range overridden per-frame later), the point is
metric; otherwise we place it at `assumed_range` along the bearing and let re-detection
refine it as the robot approaches (qr_goal re-issues when the target moves). Bearing is
always exact, which is what actually steers the walk.

Keeping the detector in its own process is deliberate: torch + CUDA live in the .venv,
rclpy lives in system python 3.10 -- they don't share an interpreter (see detect.sh).
"""
import json
import math
import struct
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray


def _jpeg_size(buf):
    """(w, h) from a JPEG's SOF marker -- no decode, no cv2/PIL dependency."""
    i, n = 2, len(buf)
    while i + 9 < n:
        if buf[i] != 0xFF:
            i += 1
            continue
        marker = buf[i + 1]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            h, w = struct.unpack(">HH", buf[i + 5:i + 9])
            return w, h
        seg = struct.unpack(">H", buf[i + 2:i + 4])[0]
        i += 2 + seg
    return None


class BarcodeLocate(Node):
    def __init__(self):
        super().__init__("barcode_locate")
        self.declare_parameter("det_url", "http://127.0.0.1:8090/detections.json")
        self.declare_parameter("frame_url", "http://127.0.0.1:8090/frame.jpg")
        self.declare_parameter("camera_frame", "head_camera_optical")
        self.declare_parameter("hfov_deg", 96.8)        # K1 ZED horizontal FOV (camera contract)
        self.declare_parameter("image_width", 0)        # 0 -> auto-detect from a frame.jpg
        self.declare_parameter("image_height", 0)
        self.declare_parameter("assumed_range", 1.5)    # m along the bearing until depth is wired
        self.declare_parameter("poll_hz", 8.0)
        self.declare_parameter("require_stable", True)  # only act on temporally-gated boxes
        self.declare_parameter("min_score", 0.30)
        self.declare_parameter("kinds", ["barcode", "qr"])   # which detection kinds count
        gp = lambda n: self.get_parameter(n).value
        self.det_url, self.frame_url = gp("det_url"), gp("frame_url")
        self.camera_frame = gp("camera_frame")
        self.hfov = math.radians(gp("hfov_deg"))
        self.W, self.H = int(gp("image_width")), int(gp("image_height"))
        self.assumed_range = gp("assumed_range")
        self.require_stable = gp("require_stable")
        self.min_score = gp("min_score")
        self.kinds = set(gp("kinds"))
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

        self.pt_pub = self.create_publisher(PointStamped, "qr_detection", 10)
        self.bear_pub = self.create_publisher(Float64MultiArray, "barcode_bearing", 10)
        self.create_timer(1.0 / gp("poll_hz"), self._tick)
        self._warned = False
        self.get_logger().info(
            f"barcode_locate up | polling {self.det_url} | hfov={math.degrees(self.hfov):.1f} deg "
            f"| range~{self.assumed_range:.1f} m | publishing /qr_detection + /barcode_bearing")

    def _ensure_intrinsics(self):
        if self.W > 0 and self.H > 0:
            return True
        try:
            buf = self._opener.open(self.frame_url, timeout=2).read()
            wh = _jpeg_size(buf)
            if wh:
                self.W, self.H = wh
                self.get_logger().info(f"camera frame {self.W}x{self.H} (auto)")
                return True
        except Exception:
            pass
        return False

    def _tick(self):
        if not self._ensure_intrinsics():
            if not self._warned:
                self.get_logger().info("waiting for the detect-server feed (frame.jpg) ...",
                                       throttle_duration_sec=4.0)
            return
        try:
            raw = self._opener.open(self.det_url, timeout=2).read()
            dets = json.loads(raw)
        except Exception:
            self.get_logger().info("detect-server not reachable yet ...", throttle_duration_sec=4.0)
            return
        best = None
        for d in dets:
            if d.get("kind") not in self.kinds:
                continue
            if self.require_stable and not d.get("stable"):
                continue
            if float(d.get("score", 0.0)) < self.min_score:
                continue
            if best is None or d["score"] > best["score"]:
                best = d
        if best is None:
            return

        x1, y1, x2, y2 = best["bbox"]
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        f = (self.W / 2.0) / math.tan(self.hfov / 2.0)       # px focal length from HFOV
        z = float(self.assumed_range)
        X = (u - self.W / 2.0) / f * z                       # optical: x right
        Y = (v - self.H / 2.0) / f * z                       #          y down
        Z = z                                                #          z forward

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.point.x, msg.point.y, msg.point.z = X, Y, Z
        self.pt_pub.publish(msg)

        u_norm = (u - self.W / 2.0) / (self.W / 2.0)         # [-1,1], left..right, for head tracking
        self.bear_pub.publish(Float64MultiArray(data=[float(u_norm), float(best["score"])]))
        self.get_logger().info(
            f"barcode {best['kind']} score={best['score']:.2f} u_norm={u_norm:+.2f} "
            f"-> cam ({X:+.2f},{Y:+.2f},{Z:+.2f})", throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = BarcodeLocate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
