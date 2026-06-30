#!/usr/bin/env python3
"""Locate the cart the robot can see, and publish it for Nav2 to walk to.

Torch-free ROS node (system python). Polls the cart detect-server (cart_detect_server.py,
YOLO-World, in the navila env) at :8090/detections.json, takes the best *stable* cart box,
and turns it into:

  /cart_detection  (geometry_msgs/PointStamped, head_camera_optical) -> qr_goal -> standoff goal
  /cart_bearing    (std_msgs/Float64MultiArray [u_norm, score])       -> head_search centres on it

Bearing (where to turn) is exact from the box centre. Range (how far) is estimated from the
cart's apparent HEIGHT vs its known real height -- height is far more view-stable than width
for a cart (front vs side changes width hugely, height barely). As the robot approaches, the
box grows -> range shrinks -> qr_goal re-issues the standoff, so the final stop distance is
self-correcting even without metric depth. (Wire true depth later for a one-shot exact range.)

Mirrors barcode_locate.py; the only new bit is height-based ranging.
"""
import hashlib
import json
import math
import struct
import time
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray


def _jpeg_size(buf):
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


class CartLocate(Node):
    def __init__(self):
        super().__init__("cart_locate")
        self.declare_parameter("det_url", "http://127.0.0.1:8090/detections.json")
        self.declare_parameter("frame_url", "http://127.0.0.1:8090/frame.jpg")
        self.declare_parameter("camera_frame", "head_camera_optical")
        self.declare_parameter("hfov_deg", 96.8)
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("real_height", 0.90)      # cart height (m), wheels->handle ~0.9
        self.declare_parameter("range_min", 0.4)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("assumed_range", 1.5)     # fallback if the box height is unusable
        self.declare_parameter("poll_hz", 8.0)
        self.declare_parameter("require_stable", True)
        self.declare_parameter("min_score", 0.05)
        self.declare_parameter("kinds", ["cart"])
        self.declare_parameter("stale_thresh", 1.5)      # s the camera FRAME may be frozen before we stop publishing
        gp = lambda n: self.get_parameter(n).value
        self.det_url, self.frame_url = gp("det_url"), gp("frame_url")
        self.camera_frame = gp("camera_frame")
        self.hfov = math.radians(gp("hfov_deg"))
        self.W, self.H = int(gp("image_width")), int(gp("image_height"))
        self.real_height = gp("real_height")
        self.range_min, self.range_max = gp("range_min"), gp("range_max")
        self.assumed_range = gp("assumed_range")
        self.require_stable = gp("require_stable")
        self.min_score = gp("min_score")
        self.kinds = set(gp("kinds"))
        self.stale_thresh = gp("stale_thresh")
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._last_z = None      # last TRUSTED (un-clipped, in-band) range, for fallback
        self._last_frame_hash = None     # for the freshness gate (the reliable feed republishes stale JPEGs)
        self._frame_frozen_since = None

        self.pt_pub = self.create_publisher(PointStamped, "cart_detection", 10)
        self.bear_pub = self.create_publisher(Float64MultiArray, "cart_bearing", 10)
        self.create_timer(1.0 / gp("poll_hz"), self._tick)
        self.get_logger().info(
            f"cart_locate up | polling {self.det_url} | hfov={math.degrees(self.hfov):.1f} deg "
            f"| real_height={self.real_height:.2f} m | publishing /cart_detection + /cart_bearing")

    def _ensure_intrinsics(self):
        if self.W > 0 and self.H > 0:
            return True
        try:
            wh = _jpeg_size(self._opener.open(self.frame_url, timeout=2).read())
            if wh:
                self.W, self.H = wh
                self.get_logger().info(f"camera frame {self.W}x{self.H} (auto)")
                return True
        except Exception:
            pass
        return False

    def _tick(self):
        if not self._ensure_intrinsics():
            self.get_logger().info("waiting for the cart detect-server feed ...",
                                   throttle_duration_sec=4.0)
            return
        # FRESHNESS GATE: the reliable /booster_video_stream REPUBLISHES the same JPEG during slow
        # windows; acting on a frozen frame causes drift/overshoot. If the frame content hasn't changed
        # for stale_thresh s, publish NOTHING -> cart_track lost-times-out and STOPS (fail-safe, not drift).
        try:
            fh = hashlib.md5(self._opener.open(self.frame_url, timeout=2).read()).hexdigest()
        except Exception:
            fh = None
        nowm = time.monotonic()
        if fh is not None and fh == self._last_frame_hash:
            frozen = nowm - (self._frame_frozen_since or nowm)
        else:
            self._last_frame_hash = fh; self._frame_frozen_since = nowm; frozen = 0.0
        if frozen > self.stale_thresh:
            self.get_logger().info(
                f"camera feed FROZEN {frozen:.1f}s (identical frame) -> not publishing; cart_track will lost-STOP",
                throttle_duration_sec=2.0)
            return
        try:
            dets = json.loads(self._opener.open(self.det_url, timeout=2).read())
        except Exception:
            self.get_logger().info("cart detect-server not reachable yet ...",
                                   throttle_duration_sec=4.0)
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
        bbox_h = max(1.0, y2 - y1)
        f = (self.W / 2.0) / math.tan(self.hfov / 2.0)      # px focal length (f_x ~= f_y)
        # range from apparent HEIGHT -- but ONLY trust it when the box is not vertically clipped.
        # A cart cut off by the top/bottom frame edge has a truncated height -> range blows up
        # (the root cause of the 1-5 m jumps). When clipped, or out of the sane band, fall back to
        # the last TRUSTED range (else the assumed range) instead of emitting a phantom distance.
        clipped = (y1 <= 1) or (y2 >= self.H - 1)
        z = f * self.real_height / bbox_h
        if clipped or not (self.range_min <= z <= self.range_max):
            z = self._last_z if self._last_z is not None else float(self.assumed_range)
            if clipped:
                self.get_logger().info("cart box clipped at frame edge -> range untrusted, using fallback",
                                       throttle_duration_sec=3.0)
        else:
            self._last_z = z
        X = (u - self.W / 2.0) / f * z                      # optical: x right, y down, z forward
        Y = (v - self.H / 2.0) / f * z
        Z = z

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.point.x, msg.point.y, msg.point.z = X, Y, Z
        self.pt_pub.publish(msg)

        u_norm = (u - self.W / 2.0) / (self.W / 2.0)
        self.bear_pub.publish(Float64MultiArray(data=[float(u_norm), float(best["score"])]))
        self.get_logger().info(
            f"cart score={best['score']:.2f} u_norm={u_norm:+.2f} range~{z:.2f} m "
            f"-> cam ({X:+.2f},{Y:+.2f},{Z:+.2f})", throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = CartLocate()
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
