import sys, time, math, argparse
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

# ─── Config ───────────────────────────────────────────────────────────────────
MODEL_PATH        = "yolov10s.pt"
DANGER_DISTANCE_M = 1.5
CAUTION_DISTANCE_M = 2.5
ZONE_TOP_PERC     = 0.25
ZONE_BOT_PERC     = 0.40
H_REAL = {'person': 1.7, 'car': 1.5, 'bus': 3.0, 'truck': 3.2, 'Unknown': 1.5}
a = 0.3

# ─── ArgParse ─────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--source', default='video', choices=['zed', 'video'],
                    help='Video source type: "zed" for ZED camera, "video" for local video or camera index')
parser.add_argument('--video', help='Path to video file or integer camera ID')
args = parser.parse_args()

# ─── Load YOLO Model ──────────────────────────────────────────────────────────
print("Loading YOLO model...")
model = YOLO(MODEL_PATH)

# ─── Zone Functions ───────────────────────────────────────────────────────────
def trapezoid_zone_pts(W, H):
    global a
    top_w = W * ZONE_TOP_PERC / 2
    bot_w = W * ZONE_BOT_PERC / 2
    y_top = int(H * a)
    y_bot = H
    cx = W / 2
    return np.array([[cx - top_w, y_top], [cx + top_w, y_top],
                     [cx + bot_w, y_bot], [cx - bot_w, y_bot]], np.int32)

# ─── Kalman Filter ────────────────────────────────────────────────────────────
class KF:
    def __init__(self, q=1e-4, r=0.2):
        self.x = 0
        self.p = 1
        self.q = q
        self.r = r

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x

# ─── Setup Video or ZED Source ────────────────────────────────────────────────
cap = None
frame = None
W, H = 0, 0
right_frame = None

# Focal length and baseline from ZED /camera_info
focal_px = 264.3975830078125
baseline_m = 31.736146926879883 / focal_px

use_cuda = False
stereo_cuda = None

if args.source == 'zed':
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class ZEDStereoSubscriber(Node):
        def __init__(self):
            super().__init__('zed_stereo_subscriber')
            self.bridge = CvBridge()
            self.left_frame = None
            self.right_frame = None

            self.sub_left = self.create_subscription(
                Image,
                '/zed/zed_node/left_gray/image_rect_gray',
                self.left_callback,
                10)

            self.sub_right = self.create_subscription(
                Image,
                '/zed/zed_node/right_gray/image_rect_gray',
                self.right_callback,
                10)

        def left_callback(self, msg):
            self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        def right_callback(self, msg):
            self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    rclpy.init()
    zed_node = ZEDStereoSubscriber()

    print("Waiting for stereo frames...")
    while rclpy.ok() and (zed_node.left_frame is None or zed_node.right_frame is None):
        rclpy.spin_once(zed_node, timeout_sec=0.1)

    frame = cv2.cvtColor(zed_node.left_frame, cv2.COLOR_GRAY2BGR)
    H, W = frame.shape[:2]

    try:
        stereo_cuda = cv2.cuda_StereoBM_create(numDisparities=64, blockSize=9)
        use_cuda = True
        print("Using CUDA StereoBM")
    except:
        print("CUDA StereoBM not available. Falling back to CPU")

else:
    if args.video is not None:
        try:
            idx = int(args.video)
            cap = cv2.VideoCapture(idx)
        except:
            cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)
    assert cap.isOpened(), "Cannot open source"
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# ─── Zone Mask and Stereo Matcher ─────────────────────────────────────────────
ZONE_PTS = trapezoid_zone_pts(W, H)
ZONE_MASK = np.zeros((H, W), np.uint8)
cv2.fillPoly(ZONE_MASK, [ZONE_PTS], 255)

stereo_cpu = cv2.StereoSGBM_create(minDisparity=0,
                                   numDisparities=64,
                                   blockSize=5,
                                   P1=8 * 5 * 5,
                                   P2=32 * 5 * 5,
                                   mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

# ─── Main Loop ────────────────────────────────────────────────────────────────
def main_loop():
    kalman, last_dist, prev_time = {}, {}, time.time()
    global frame, cap, zed_node

    while True:
        if args.source == 'zed':
            if not rclpy.ok():
                break
            rclpy.spin_once(zed_node, timeout_sec=0.01)
            left = zed_node.left_frame
            right = zed_node.right_frame
            if left is None or right is None:
                continue
            frame = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                break
            left = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            right = left.copy()

        # Stereo Depth (ZED only)
        if args.source == 'zed':
            if use_cuda:
                l_gpu = cv2.cuda_GpuMat()
                r_gpu = cv2.cuda_GpuMat()
                l_gpu.upload(left)
                r_gpu.upload(right)
                d_gpu = stereo_cuda.compute(l_gpu, r_gpu)
                disparity = d_gpu.download().astype(np.float32) / 16.0
            else:
                disparity = stereo_cpu.compute(left, right).astype(np.float32) / 16.0

            disparity[disparity <= 0.1] = 0.1
            depth_map = (focal_px * baseline_m) / disparity
        else:
            depth_map = None

        writable_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
        dt = time.time() - prev_time
        prev_time = time.time()
        ann = Annotator(writable_frame, line_width=2)
        scene_state = "CLEAR"

        for r in model.track(writable_frame, persist=True, iou=0.7, conf=0.4, verbose=False):
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                tid = int(b.id.cpu()) if b.id is not None else -1
                cls = int(b.cls.cpu())
                name = model.names[cls]

                # ─── Distance Estimation ───────────────────────────────────────
                if args.source == 'zed' and depth_map is not None:
                    if 0 <= cy < depth_map.shape[0] and 0 <= cx < depth_map.shape[1]:
                        window = depth_map[max(0, cy-2):cy+3, max(0, cx-2):cx+3]
                        dist = float(np.median(window))
                    else:
                        dist = 0.0
                else:
                    h_pix = max(1, y2 - y1)
                    dist = H_REAL.get(name, 1.5) * focal_px / h_pix

                # ─── Kalman + Velocity ────────────────────────────────────────
                if tid not in kalman:
                    kalman[tid] = KF()
                dist = kalman[tid].update(dist)
                vel = (dist - last_dist.get(tid, dist)) / dt if dt > 0 else 0
                last_dist[tid] = dist

                in_danger_zone = ZONE_MASK[cy, cx] > 0
                if in_danger_zone:
                    if dist <= DANGER_DISTANCE_M:
                        scene_state = "STOP"
                    elif dist <= CAUTION_DISTANCE_M and scene_state != "STOP":
                        scene_state = "CAUTION"

                color = (0, 0, 255) if in_danger_zone else (255, 0, 0)
                ann.box_label((x1, y1, x2, y2), "", color=color)

                label_text = f"{dist:.1f}m | {vel:+.2f}m/s"
                (text_w, text_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                label_x = x1 + 4
                label_y = y2 + text_h + 5 if y2 + text_h + 10 < H else y1 - 5
                rect_top = y2 if y2 + text_h + 10 < H else y1 - text_h - 10
                rect_bottom = rect_top + text_h + 10
                rect_right = x1 + text_w + 8

                cv2.rectangle(writable_frame, (x1, rect_top), (rect_right, rect_bottom), color, -1)
                cv2.putText(writable_frame, label_text, (label_x, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.polylines(writable_frame, [ZONE_PTS], isClosed=True, color=(0, 255, 0), thickness=2)
        banner_color = (0, 255, 0) if scene_state == "CLEAR" else (255, 255, 0) if scene_state == "CAUTION" else (255, 0, 0)
        cv2.rectangle(writable_frame, (0, 0), (W, 30), banner_color, -1)
        cv2.putText(writable_frame, f" NAV STATE: {scene_state} ", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        frame_to_show = ann.result()
        frame_to_show_bgr = cv2.cvtColor(frame_to_show, cv2.COLOR_RGB2BGR)

        scale = 2.5
        frame_to_show_bgr = cv2.resize(frame_to_show_bgr, (int(W * scale), int(H * scale)))

        cv2.imshow("AMR Safe-Navigation View", frame_to_show_bgr)
        if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
            break

# ─── Run ──────────────────────────────────────────────────────────────────────
try:
    main_loop()
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    if cap:
        cap.release()
    if args.source == 'zed':
        rclpy.shutdown()
    cv2.destroyAllWindows()