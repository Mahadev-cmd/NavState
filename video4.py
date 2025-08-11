import sys, time, math, argparse
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

# ─── Config ───────────────────────────────────────────────────────────────────
MODEL_PATH        = "yolov10s.pt"
DANGER_DISTANCE_M = 2.0
CAUTION_DISTANCE_M = 2.5
ZONE_TOP_PERC     = 0.30
ZONE_BOT_PERC     = 0.60
H_REAL = {'person': 1.7, 'car': 1.5, 'bus': 3.0, 'truck': 3.2, 'Unknown': 1.5}
a = 0.7
# Resize frame before showing
scale = 1.0  # Increase this scale for larger size

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

def box_intersects_trapezoid(x1, y1, x2, y2, zone_pts):
    """Check if bounding box intersects with trapezoid zone lines"""
    # Convert to float to ensure proper data types
    x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
    
    # Create box corners as float tuples
    box_corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    
    # Check if any box corner is inside the trapezoid
    for corner in box_corners:
        if cv2.pointPolygonTest(zone_pts.astype(np.float32), corner, False) >= 0:
            return True
    
    # Check if any trapezoid corner is inside the box
    for corner in zone_pts:
        corner_x, corner_y = float(corner[0]), float(corner[1])
        if x1 <= corner_x <= x2 and y1 <= corner_y <= y2:
            return True
    
    # Check for line intersections between box edges and trapezoid edges
    def line_intersect(p1, p2, p3, p4):
        """Check if line segments p1p2 and p3p4 intersect"""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)
    
    # Box edges as tuples of floats
    box_edges = [
        (box_corners[0], box_corners[1]),  # top
        (box_corners[1], box_corners[2]),  # right
        (box_corners[2], box_corners[3]),  # bottom
        (box_corners[3], box_corners[0])   # left
    ]
    
    # Trapezoid edges as tuples of floats
    trap_edges = []
    for i in range(len(zone_pts)):
        p1 = (float(zone_pts[i][0]), float(zone_pts[i][1]))
        p2 = (float(zone_pts[(i + 1) % len(zone_pts)][0]), float(zone_pts[(i + 1) % len(zone_pts)][1]))
        trap_edges.append((p1, p2))
    
    # Check all edge intersections
    for box_edge in box_edges:
        for trap_edge in trap_edges:
            if line_intersect(box_edge[0], box_edge[1], trap_edge[0], trap_edge[1]):
                return True
    
    return False

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

# ─── Setup Video Source ───────────────────────────────────────────────────────
cap = None
frame = None
W, H = 0, 0

if args.source == 'zed':
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class ZEDVideoSubscriber(Node):
        def __init__(self):
            super().__init__('zed_video_subscriber')
            self.bridge = CvBridge()
            self.frame = None
            self.subscription = self.create_subscription(
                Image,
                '/zed/zed_node/left_gray/image_rect_gray',
                self.listener_callback,
                10
            )
        def listener_callback(self, msg):
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    rclpy.init()
    zed_node = ZEDVideoSubscriber()

    print("Waiting for first ZED frame...")
    while rclpy.ok() and zed_node.frame is None:
        rclpy.spin_once(zed_node, timeout_sec=0.1)

    frame = zed_node.frame
    H, W = frame.shape[:2]

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

ZONE_PTS = trapezoid_zone_pts(W, H)
ZONE_MASK = np.zeros((H, W), np.uint8)
cv2.fillPoly(ZONE_MASK, [ZONE_PTS], 255)

focal_px = 265  # Use K[0] or K[4] from camera_info

# ─── Main Loop ────────────────────────────────────────────────────────────────
def main_loop():
    global z
    kalman, last_dist, prev_time = {}, {}, time.time()
    global frame, cap, zed_node

    while True:
        if args.source == 'zed':
            if not rclpy.ok():
                break
            rclpy.spin_once(zed_node, timeout_sec=0.01)
            frm = zed_node.frame
            if frm is None:
                continue
        else:
            ret, frm = cap.read()
            if not ret:
                break

        writable_frame = cv2.cvtColor(frm.copy(), cv2.COLOR_BGR2RGB)
        dt = time.time() - prev_time
        prev_time = time.time()

        ann = Annotator(writable_frame, line_width=2)
        scene_state = "CLEAR"

        for r in model.track(writable_frame, persist=True, iou=0.7, conf=0.4, verbose=False):
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                tid = int(b.id.cpu()) if b.id is not None else -1
                cls = int(b.cls.cpu())
                name = model.names[cls]

                h_pix = max(1, y2 - y1)
                dist = H_REAL.get(name, 1.5) * focal_px / h_pix

                if tid not in kalman:
                    kalman[tid] = KF()
                dist = kalman[tid].update(dist)

                vel = 0
                if tid in last_dist and dt > 0:
                    vel = (dist - last_dist[tid]) / dt
                last_dist[tid] = dist

                # Check if object center is in danger zone OR if bounding box intersects trapezoid
                in_danger_zone = ZONE_MASK[int(cy), int(cx)] > 0
                box_intersects_zone = box_intersects_trapezoid(x1, y1, x2, y2, ZONE_PTS)
                
                # Trigger danger/caution if distance conditions are met AND (center in zone OR box intersects zone)
                if (in_danger_zone or box_intersects_zone):
                    if dist <= DANGER_DISTANCE_M:
                        scene_state = "STOP"
                elif dist <= CAUTION_DISTANCE_M and scene_state != "STOP":
                    scene_state = "CAUTION"

                # Color coding: red if in danger zone or intersects, blue otherwise
                color = (255, 0, 0) if (in_danger_zone or box_intersects_zone) else (0, 0, 255)
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
        
        # cv2.polylines(writable_frame, [ZONE_PTS], isClosed=True, color=(255, 255, 0), thickness=2)
        
        frame_to_show = ann.result()
        frame_to_show_bgr = cv2.cvtColor(frame_to_show, cv2.COLOR_RGB2BGR)

        
        height, width = frame_to_show_bgr.shape[:2]
        frame_to_show_bgr = cv2.resize(frame_to_show_bgr, (int(width * scale), int(height * scale)))

        # Create status banner separately (reduced height from 30 to 20)
        banner_height = 21
        banner_color = (0, 255, 0) if scene_state == "CLEAR" else (0, 255, 255) if scene_state == "CAUTION" else (0, 0, 255)
        banner = np.full((banner_height, frame_to_show_bgr.shape[1], 3), banner_color, dtype=np.uint8)
        
        # Add text to banner (reduced font scale from 0.7 to 0.4)
        cv2.putText(banner, f" NAV STATE: {scene_state} ", (10, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        # Combine banner and frame vertically
        combined_frame = np.vstack((banner, frame_to_show_bgr))

        cv2.imshow("AMR Safe-Navigation View", combined_frame)

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
        import rclpy
        rclpy.shutdown()
    cv2.destroyAllWindows()
