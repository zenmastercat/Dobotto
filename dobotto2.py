#!/usr/bin/env python3

import os
os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["QT_LOGGING_RULES"] = "*=false"

import cv2
import glob
import time
import numpy as np
import pydobot
from serial.tools import list_ports

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image


class FastDobotStackerNode(Node):
    """
    ROS 2 High-Speed Stacking Node with Instant Out-of-Range Color Switching.
    """
    def __init__(self):
        super().__init__('fast_dobot_stacker')

        # --------------------------------------------------------------
        # 1. ROS 2 PARAMETERS
        # --------------------------------------------------------------
        self.declare_parameter('arm_speed', 200)             # Max Speed (mm/s)
        self.declare_parameter('arm_accel', 200)             # Max Acceleration (mm/s²)
        self.declare_parameter('safe_z', 25.0)               # Clearance Z (mm)
        self.declare_parameter('base_pick_z', -45.0)         # Pick Z (mm)
        self.declare_parameter('block_height', 25.0)         # Block height (mm)
        self.declare_parameter('target_tower_height', 4)     # Number of blocks
        self.declare_parameter('camera_index', -1)           # Auto-detect if -1

        self.arm_speed = self.get_parameter('arm_speed').value
        self.arm_accel = self.get_parameter('arm_accel').value
        self.safe_z = self.get_parameter('safe_z').value
        self.base_pick_z = self.get_parameter('base_pick_z').value
        self.block_height = self.get_parameter('block_height').value
        self.target_tower_height = self.get_parameter('target_tower_height').value

        # Motion & Hardware Limits
        self.adaptive_pick_z_offset = 0.0
        self.suction_dwell = 0.15  # Fast suction engage time (s)
        self.MIN_ROBOT_REACH = 140.0 # Min radial reach limit (mm)
        self.MAX_ROBOT_REACH = 330.0 # Max radial reach limit (mm)

        # Default Stack Zone
        self.stack_x = 220.0
        self.stack_y = -100.0

        # --------------------------------------------------------------
        # 2. ROS 2 PUBLISHERS
        # --------------------------------------------------------------
        self.status_pub = self.create_publisher(String, '/dobot/status', 10)
        self.image_pub = self.create_publisher(Image, '/dobot/camera/image_raw', 10)

        # --------------------------------------------------------------
        # 3. HOMOGRAPHY CALIBRATION MATRICES
        # --------------------------------------------------------------
        self.src_pixels = np.array([
            [100, 100], [540, 100], [540, 380], [100, 380]
        ], dtype=np.float32)

        self.dst_robot = np.array([
            [150.0, 120.0], [150.0, -120.0], [320.0, -120.0], [320.0, 120.0]
        ], dtype=np.float32)

        self.H_matrix, _ = cv2.findHomography(self.src_pixels, self.dst_robot)
        self.H_inverse, _ = cv2.findHomography(self.dst_robot, self.src_pixels)

        # Color Definitions
        self.color_bgr = {
            'red': (0, 0, 255), 'green': (0, 255, 0),
            'blue': (255, 0, 0), 'yellow': (0, 255, 255)
        }
        self.color_ranges = {}

        # Hardware Initialization
        self.cap = self._init_camera(self.get_parameter('camera_index').value)
        self.device = self._init_dobot()

        self._calibrate_colors()
        self.stack_x, self.stack_y = self._find_empty_stack_zone()

    # ==================================================================
    # HARDWARE & ROS HELPER METHODS
    # ==================================================================

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"[ROS2 STATUS] {text}")

    def publish_frame(self, frame):
        if frame is None:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dobot_camera"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

    def _init_camera(self, override_idx):
        if override_idx != -1:
            cap = cv2.VideoCapture(override_idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                return cap

        for idx in [2, 4, 6, 0]:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ret, frame = cap.read()
                if ret and frame is not None and frame.mean() > 10.0:
                    self.get_logger().info(f"Connected to camera on index {idx}")
                    return cap
                cap.release()
        raise RuntimeError("No operational V4L2 camera detected.")

    def _init_dobot(self):
        ports = list_ports.comports()
        dobot_port = None
        for p in ports:
            if any(k in p.device for k in ["USB", "ACM", "ttyUSB", "ttyACM"]):
                dobot_port = p.device
                break

        if not dobot_port:
            raise RuntimeError("No Dobot Magician found on serial USB ports.")

        self.get_logger().info(f"Connecting to Dobot on port: {dobot_port}")
        device = pydobot.Dobot(port=dobot_port, verbose=False)
        device.speed(self.arm_speed, self.arm_accel)

        self.publish_status("Homing Dobot arm...")
        if hasattr(device, 'home'):
            device.home()
        time.sleep(12)
        device.move_to(200.0, 0.0, self.safe_z, 0.0, wait=False)
        return device

    # ==================================================================
    # KINEMATICS & VISION
    # ==================================================================

    def pixels_to_robot(self, px, py):
        pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
        tf = cv2.perspectiveTransform(pt, self.H_matrix)
        return float(tf[0][0][0]), float(tf[0][0][1])

    def robot_to_pixels(self, rx, ry):
        pt = np.array([[[float(rx), float(ry)]]], dtype=np.float32)
        tf = cv2.perspectiveTransform(pt, self.H_inverse)
        return int(round(tf[0][0][0])), int(round(tf[0][0][1]))

    def get_roi_mask(self, shape):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.src_pixels.astype(np.int32)], 255)
        return mask

    def _calibrate_colors(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            min_s, min_v = 25, 20
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            roi_mask = self.get_roi_mask(frame.shape)
            s_channel = hsv[:, :, 1][roi_mask == 255]
            v_channel = hsv[:, :, 2][roi_mask == 255]
            min_s = int(np.percentile(s_channel[s_channel > 15], 10)) if len(s_channel) > 0 else 25
            min_v = int(np.percentile(v_channel[v_channel > 15], 10)) if len(v_channel) > 0 else 20
            min_s, min_v = max(20, min(min_s, 60)), max(20, min(min_v, 60))

        self.color_ranges = {
            'red': [((0, min_s, min_v), (15, 255, 255)), ((160, min_s, min_v), (180, 255, 255))],
            'yellow': [((13, min_s, min_v), (35, 255, 255))],
            'green': [((36, min_s, min_v), (85, 255, 255))],
            'blue': [((86, min_s, min_v), (140, 255, 255))]
        }

    def _find_empty_stack_zone(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return 220.0, -100.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        roi_mask = self.get_roi_mask(frame.shape)
        combined_mask = np.zeros(hsv.shape[:2], dtype="uint8")

        for color_name, ranges in self.color_ranges.items():
            for lower, upper in ranges:
                combined_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

        inv_mask = cv2.bitwise_not(combined_mask)
        workspace_open = cv2.bitwise_and(inv_mask, inv_mask, mask=roi_mask)
        dist_transform = cv2.distanceTransform(workspace_open, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist_transform)

        if max_val > 25.0:
            return self.pixels_to_robot(max_loc[0], max_loc[1])
        return 220.0, -100.0

    def find_target_block(self, target_color):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        roi_mask = self.get_roi_mask(frame.shape)
        mask = np.zeros(hsv.shape[:2], dtype="uint8")

        for lower, upper in self.color_ranges.get(target_color, []):
            mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

        mask = cv2.bitwise_and(mask, roi_mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        stack_px, stack_py = self.robot_to_pixels(self.stack_x, self.stack_y)

        for c in contours:
            if cv2.contourArea(c) > 300:
                rect = cv2.minAreaRect(c)
                (cX, cY), (w, h), angle = rect

                if np.hypot(cX - stack_px, cY - stack_py) < 40:
                    continue

                if w < h:
                    angle += 90.0

                box = np.int32(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, self.color_bgr.get(target_color, (0, 255, 0)), 2)
                self.publish_frame(frame)

                rx, ry = self.pixels_to_robot(cX, cY)
                return rx, ry, angle, cX, cY

        self.publish_frame(frame)
        return None

    def is_out_of_range_or_missing(self, target_color):
        """
        Checks if the target color is out of camera view OR out of physical arm reach.
        Returns (is_invalid, target_info_tuple)
        """
        target_info = self.find_target_block(target_color)
        if target_info is None:
            return True, None  # Color missing from camera frame

        rx, ry, angle, cX, cY = target_info
        radial_distance = np.hypot(rx, ry)

        # Check physical robot kinematic reach limits
        if radial_distance < self.MIN_ROBOT_REACH or radial_distance > self.MAX_ROBOT_REACH or rx < 110.0:
            return True, target_info  # Detected by camera, but physically out of range

        return False, target_info

    def verify_pickup_failed(self, pick_px, pick_py, target_color):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return False

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pickup_mask = np.zeros(hsv.shape[:2], dtype="uint8")
        cv2.circle(pickup_mask, (int(pick_px), int(pick_py)), 35, 255, -1)

        target_mask = np.zeros(hsv.shape[:2], dtype="uint8")
        for lower, upper in self.color_ranges.get(target_color, []):
            target_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

        remaining_pixels = cv2.bitwise_and(target_mask, pickup_mask)
        return np.count_nonzero(remaining_pixels) > 120

    # ==================================================================
    # FAST MOTION & DYNAMIC QUEUE PIPELINE
    # ==================================================================

    def fast_move(self, x, y, z, r=0.0):
        self.device.move_to(x, y, z, r, wait=True)

    def execute_fast_stacking(self, initial_order):
        remaining_colors = list(initial_order)
        tower_count = 0

        while tower_count < self.target_tower_height and remaining_colors and rclpy.ok():
            target_color = remaining_colors[0]
            self.publish_status(f"Layer {tower_count+1}/{self.target_tower_height}: Checking {target_color.upper()}")

            # INSTANT OUT-OF-RANGE / MISSING CHECK
            out_of_range, target_info = self.is_out_of_range_or_missing(target_color)

            if out_of_range:
                self.publish_status(
                    f"[OUT OF RANGE / MISSING] {target_color.upper()} not reachable! "
                    f"Immediately switching to pick next color..."
                )
                # Rotate current color to back of queue and immediately switch
                skipped_color = remaining_colors.pop(0)
                remaining_colors.append(skipped_color)
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.001)
                continue

            rx, ry, angle, pick_px, pick_py = target_info

            # FAST PICKUP SEQUENCE
            self.publish_status(f"Picking {target_color.upper()} at ({rx:.1f}, {ry:.1f})")
            
            self.fast_move(rx, ry, self.safe_z, r=angle)
            effective_pick_z = self.base_pick_z + self.adaptive_pick_z_offset
            self.fast_move(rx, ry, effective_pick_z, r=angle)

            self.device.suck(True)
            time.sleep(self.suction_dwell)

            self.fast_move(rx, ry, self.safe_z, r=angle)

            # PICKUP VERIFICATION
            if self.verify_pickup_failed(pick_px, pick_py, target_color):
                self.publish_status(f"[PICKUP FAILED] {target_color.upper()} block still at location! Retrying...")
                self.device.suck(False)
                self.adaptive_pick_z_offset -= 1.0
                time.sleep(0.1)
                continue

            # FAST PLACE SEQUENCE
            current_stack_z = self.base_pick_z + (tower_count * self.block_height)
            self.publish_status(f"Placing {target_color.upper()} on stack at Z={current_stack_z:.1f}mm")

            self.fast_move(self.stack_x, self.stack_y, self.safe_z, r=0.0)
            self.fast_move(self.stack_x, self.stack_y, current_stack_z, r=0.0)

            self.device.suck(False)
            time.sleep(0.1)

            self.fast_move(self.stack_x, self.stack_y, self.safe_z, r=0.0)

            # Successfully placed -> remove from queue and increment tower
            remaining_colors.pop(0)
            tower_count += 1
            self.publish_status(f"Successfully stacked block {tower_count}/{self.target_tower_height}")

        self.publish_status("OBJECTIVE COMPLETED FAST!")
        self.fast_move(200.0, 0.0, self.safe_z + 40.0, r=0.0)


# ==================================================================
# MAIN ENTRYPOINT
# ==================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FastDobotStackerNode()

    valid_colors = ['red', 'green', 'blue', 'yellow']
    print("\n" + "="*50)
    print(" ROS 2 HIGH-SPEED DOBOT STACKER (DYNAMIC COLOR SWITCHING)")
    print("="*50)

    user_str = input("Enter 4 block colors in order (e.g., red, green, blue, yellow): ").strip().lower()
    stack_order = [c.strip() for c in user_str.split(',') if c.strip() in valid_colors]

    if len(stack_order) != 4:
        stack_order = ['red', 'green', 'blue', 'yellow']
        print(f"[DEFAULT] Using sequence: {stack_order}")

    try:
        node.execute_fast_stacking(stack_order)
    except KeyboardInterrupt:
        node.get_logger().info("Execution interrupted by user.")
    finally:
        node.device.suck(False)
        node.device.close()
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
