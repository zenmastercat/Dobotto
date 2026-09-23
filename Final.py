import cv2
import numpy as np
import time

# ==============================================================================
# 1. HARDWARE CONTROLLER & DYNAMIC CORNER TEACHING
# ==============================================================================
class RobotController:
    def __init__(self, port="/dev/ttyUSB0"):
        self.device = None
        try:
            from pydobot import Dobot
            self.device = Dobot(port=port)
            self.device.speed(100, 100)
            print(f"[INFO] Dobot connected on {port}.")
        except Exception as e:
            print(f"[WARN] Dobot hardware offline: {e}")

    def get_current_pose(self):
        """Reads current [X, Y, Z, R] directly from the Dobot arm safely across pydobot library variants."""
        if self.device is None:
            return np.array([240.0, 0.0, -34.0, 0.0], dtype=np.float32)

        # 1. Retrieve raw pose data using whichever method exists in your installed library
        if hasattr(self.device, 'get_pose'):
            pose_obj = self.device.get_pose()
        elif hasattr(self.device, 'pose') and callable(getattr(self.device, 'pose')):
            pose_obj = self.device.pose()
        elif hasattr(self.device, 'pose'):
            pose_obj = self.device.pose
        else:
            raise AttributeError("Installed Dobot library has neither 'get_pose()' nor 'pose()' method.")

        # 2. Parse coordinates from return structure
        # pydobotplus (returns object with .position.x, .position.y, etc.)
        if hasattr(pose_obj, 'position'):
            pos = pose_obj.position
            return np.array([float(pos.x), float(pos.y), float(pos.z), float(pos.r)], dtype=np.float32)

        # Object with direct attributes (.x, .y, .z, .r)
        if hasattr(pose_obj, 'x') and hasattr(pose_obj, 'y'):
            return np.array([
                float(pose_obj.x), 
                float(pose_obj.y), 
                float(pose_obj.z), 
                float(getattr(pose_obj, 'r', 0.0))
            ], dtype=np.float32)

        # Standard pydobot tuple: (x, y, z, r, j1, j2, j3, j4)
        if isinstance(pose_obj, (tuple, list, np.ndarray)):
            return np.array([float(pose_obj[0]), float(pose_obj[1]), float(pose_obj[2]), float(pose_obj[3])], dtype=np.float32)

        raise ValueError(f"Unrecognized pose format returned by Dobot library: {type(pose_obj)}")

    def move_to(self, x, y, z, r, cap=None):
        if self.device is None:
            print(f"[OFFLINE MOVE] X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}, R:{r:.2f}")
            return

        self.device.move_to(x, y, z, r)
        if cap is not None:
            pump_camera(cap, duration=1.2)
        else:
            time.sleep(1.2)

    def set_suck(self, enable=True):
        if self.device is None:
            return
        self.device.suck(enable)
        time.sleep(0.5)


def teach_robot_corners(robot):
    """
    Prompts the user to physically position the Dobot arm at the 4 farthest corners.
    Reads real-time coordinates from device.pose() to build the physical XO map.
    """
    print("\n" + "="*60)
    print(" STEP 1: PHYSICAL ROBOT ARM CORNER CALIBRATION")
    print(" Press the unlock button on the Dobot arm and move it by hand.")
    print("="*60)

    corner_names = [
        "1. Top-Left Corner     (Far Left)",
        "2. Top-Right Corner    (Far Right)",
        "3. Bottom-Left Corner  (Near Left)",
        "4. Bottom-Right Corner (Near Right)"
    ]
    
    taught_corners = []

    for name in corner_names:
        input(f"\n>> Move the Dobot arm to [{name}] and press ENTER...")
        pose = robot.get_current_pose()
        taught_corners.append(pose)
        print(f"   [SAVED] {name} -> X:{pose[0]:.2f}, Y:{pose[1]:.2f}, Z:{pose[2]:.2f}, R:{pose[3]:.2f}")

    print("\n[SUCCESS] All 4 robot corners recorded!")
    return taught_corners


def generate_xo_physical_slots(corners):
    """
    Bilinear interpolation to compute all 9 physical 3D cell coordinates:
    [ Slot 1 | Slot 2 | Slot 3 ]
    [ Slot 4 | Slot 5 | Slot 6 ]
    [ Slot 7 | Slot 8 | Slot 9 ]
    """
    tl, tr, bl, br = corners
    slots = []

    for r in range(3):      # Row 0 (Top), Row 1 (Mid), Row 2 (Bot)
        u = r / 2.0
        for c in range(3):  # Col 0 (Left), Col 1 (Center), Col 2 (Right)
            v = c / 2.0
            pos = (1 - u) * ((1 - v) * tl + v * tr) + \
                  u * ((1 - v) * bl + v * br)
            slots.append(pos)

    return slots


# ==============================================================================
# 2. VISION CONFIG & XO GRID SCANNER
# ==============================================================================
CAMERA_INDEX = 1
CAMERA_FLIP_MODE = -1
SAFE_HOVER_Z = 50.0
BEFORE_PLACE_POSE = (200.0, 0.00012, 150.0, 0.0)

COLOR_RANGES = {
    "Red": {
        "lower1": np.array([0, 130, 80]),   "upper1": np.array([10, 255, 255]),
        "lower2": np.array([170, 130, 80]), "upper2": np.array([180, 255, 255]),
        "bgr": (0, 0, 255)
    },
    "Green": {
        "lower1": np.array([35, 110, 60]),  "upper1": np.array([85, 255, 255]),
        "bgr": (0, 255, 0)
    },
    "Blue": {
        "lower1": np.array([95, 120, 60]),  "upper1": np.array([130, 255, 255]),
        "bgr": (255, 0, 0)
    },
    "Yellow": {
        "lower1": np.array([20, 120, 120]), "upper1": np.array([34, 255, 255]),
        "bgr": (0, 255, 255)
    }
}

calibration_mode = True
calib_clicks = []
xo_grid_polygons = None
board_state = ["Empty"] * 9
selected_slot_index = None


def create_xo_grid_polygons(clicks):
    """Builds the 3x3 visual bounding boxes on screen from 4 user clicks."""
    tl, tr, bl, br = [np.array(p, dtype=np.float32) for p in clicks]
    polygons = []

    for r in range(3):
        u0, u1 = r / 3.0, (r + 1) / 3.0
        for c in range(3):
            v0, v1 = c / 3.0, (c + 1) / 3.0

            p0 = (1 - u0) * ((1 - v0) * tl + v0 * tr) + u0 * ((1 - v0) * bl + v0 * br)
            p1 = (1 - u0) * ((1 - v1) * tl + v1 * tr) + u0 * ((1 - v1) * bl + v1 * br)
            p2 = (1 - u1) * ((1 - v1) * tl + v1 * tr) + u1 * ((1 - v1) * bl + v1 * br)
            p3 = (1 - u1) * ((1 - v0) * tl + v0 * tr) + u1 * ((1 - v0) * bl + v0 * br)

            polygons.append(np.array([p0, p1, p2, p3], dtype=np.int32))

    return polygons


def scan_xo_board_colors(frame, grid_polygons, min_pixels=300):
    """Scans all 9 visual slots and detects block colors inside each cell."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    states = ["Empty"] * 9

    for idx, poly in enumerate(grid_polygons):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)

        detected_color = "Empty"
        highest_pixel_count = 0

        for color_name, config in COLOR_RANGES.items():
            c_mask = cv2.inRange(hsv, config["lower1"], config["upper1"])
            if "lower2" in config:
                c_mask2 = cv2.inRange(hsv, config["lower2"], config["upper2"])
                c_mask = cv2.bitwise_or(c_mask, c_mask2)

            cell_mask = cv2.bitwise_and(c_mask, c_mask, mask=mask)
            count = cv2.countNonZero(cell_mask)

            if count > min_pixels and count > highest_pixel_count:
                highest_pixel_count = count
                detected_color = color_name

        states[idx] = detected_color

    return states


# ==============================================================================
# 3. INTERACTIVE MOUSE & KEYBOARD CONTROLS
# ==============================================================================
def on_mouse_click(event, x, y, flags, param):
    global calibration_mode, calib_clicks, xo_grid_polygons, selected_slot_index, board_state

    if event == cv2.EVENT_LBUTTONDOWN:
        # Camera Calibration: Align 4 visual corners on screen
        if calibration_mode:
            calib_clicks.append([x, y])
            labels = ["Slot 1 (TL)", "Slot 3 (TR)", "Slot 7 (BL)", "Slot 9 (BR)"]
            print(f"[CAM CALIB] Corner {len(calib_clicks)}/4 ({labels[len(calib_clicks)-1]}) set at ({x}, {y})")

            if len(calib_clicks) == 4:
                xo_grid_polygons = create_xo_grid_polygons(calib_clicks)
                calibration_mode = False
                print("[READY] Camera aligned to physical XO table!")
            return

        # Pick Selection: User clicks a slot on the video stream
        if xo_grid_polygons is not None:
            for idx, poly in enumerate(xo_grid_polygons):
                if cv2.pointPolygonTest(poly, (x, y), False) >= 0:
                    color = board_state[idx]
                    if color != "Empty":
                        selected_slot_index = idx
                        print(f"\n[SELECTED CELL {idx+1}] Targeted {color} Block")
                    else:
                        print(f"\n[SELECTED CELL {idx+1}] Cell is Empty")
                    return


def trigger_color_pick(target_color):
    global selected_slot_index, board_state
    for idx, color in enumerate(board_state):
        if color.lower() == target_color.lower():
            selected_slot_index = idx
            print(f"\n[HOTKEY] Found {target_color} block at Cell {idx+1}")
            return
    print(f"[WARN] No {target_color} block detected on the board!")


def pump_camera(cap, duration=1.0):
    end_time = time.time() + duration
    while time.time() < end_time:
        ret, frame = cap.read()
        if ret and frame is not None:
            if CAMERA_FLIP_MODE is not None:
                frame = cv2.flip(frame, CAMERA_FLIP_MODE)
            cv2.imshow("XO Board View", frame)
            cv2.waitKey(1)


# ==============================================================================
# 4. MAIN PROGRAM FLOW
# ==============================================================================
def main():
    global calibration_mode, calib_clicks, xo_grid_polygons, board_state, selected_slot_index

    # 1. Initialize Robot Hardware
    robot = RobotController(port="/dev/ttyUSB0")

    # 2. Teach Robot Corners directly from device.pose()
    raw_corners = teach_robot_corners(robot)
    physical_slots = generate_xo_physical_slots(raw_corners)

    # Calculate stacking drop locations using the taught center cell (Slot 5)
    center_cell_pos = physical_slots[4]
    stack_target_positions = [
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + 0.0,  0.0),  # Level 1
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + 24.0, 0.0),  # Level 2
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + 48.0, 0.0),  # Level 3
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + 72.0, 0.0)   # Level 4
    ]

    # 3. Start Camera Video Feed
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)

    stacked_count = 0
    MAX_STACK = len(stack_target_positions)

    cv2.namedWindow("XO Board View")
    cv2.setMouseCallback("XO Board View", on_mouse_click)

    print("\n" + "="*60)
    print(" STEP 2: CAMERA ALIGNMENT CALIBRATION")
    print(" Click the 4 corners on the camera view window in order:")
    print("   1. Slot 1 (Top-Left)     2. Slot 3 (Top-Right)")
    print("   3. Slot 7 (Bottom-Left)  4. Slot 9 (Bottom-Right)")
    print("="*60 + "\n")

    calib_labels = ["1. Slot 1 (TL)", "2. Slot 3 (TR)", "3. Slot 7 (BL)", "4. Slot 9 (BR)"]

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        if CAMERA_FLIP_MODE is not None:
            frame = cv2.flip(frame, CAMERA_FLIP_MODE)

        display_frame = frame.copy()

        # CAMERA CALIBRATION OVERLAY
        if calibration_mode:
            step = len(calib_clicks)
            if step < 4:
                cv2.putText(display_frame, f"CLICK CAMERA: {calib_labels[step]}",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            for idx, pt in enumerate(calib_clicks):
                cv2.circle(display_frame, tuple(pt), 6, (0, 255, 255), -1)
                cv2.putText(display_frame, calib_labels[idx], (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # MAIN OPERATIONAL OVERLAY & COLOR SCANNING
        else:
            board_state = scan_xo_board_colors(frame, xo_grid_polygons)

            cv2.putText(display_frame, f"Stacked: {stacked_count}/{MAX_STACK} | Click Cell or Press [r,g,b,y]",
                        (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            for idx, poly in enumerate(xo_grid_polygons):
                color_name = board_state[idx]
                bgr = COLOR_RANGES[color_name]["bgr"] if color_name in COLOR_RANGES else (80, 80, 80)

                cv2.polylines(display_frame, [poly], isClosed=True, color=bgr, thickness=2)

                M = cv2.moments(poly)
                if M["m00"] != 0:
                    mcx = int(M["m10"] / M["m00"])
                    mcy = int(M["m01"] / M["m00"])
                    cv2.putText(display_frame, f"[{idx+1}] {color_name}", (mcx - 30, mcy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # EXECUTE PICK & PLACE SEQUENCE
        if selected_slot_index is not None and not calibration_mode:
            if stacked_count < MAX_STACK:
                slot_idx = selected_slot_index
                color = board_state[slot_idx]

                pick_x, pick_y, pick_z, pick_r = physical_slots[slot_idx]
                place_x, place_y, place_z, place_r = stack_target_positions[stacked_count]

                print(f"\n[ACTION] Picking {color} Block from Cell {slot_idx+1}")
                print(f"         Taught Slot Pos: X:{pick_x:.2f}, Y:{pick_y:.2f}, Z:{pick_z:.2f}")

                # Move sequence
                robot.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)
                robot.move_to(pick_x, pick_y, pick_z, pick_r, cap)
                robot.set_suck(True)
                robot.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)

                robot.move_to(*BEFORE_PLACE_POSE, cap=cap)
                robot.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
                robot.move_to(place_x, place_y, place_z, place_r, cap)
                robot.set_suck(False)
                robot.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
                robot.move_to(*BEFORE_PLACE_POSE, cap=cap)

                stacked_count += 1
            else:
                print("[INFO] Maximum stack limit reached!")

            selected_slot_index = None

        cv2.imshow("XO Board View", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            print("\n[RE-ALIGN CAMERA] Click the 4 corners of the XO board on screen.")
            calibration_mode = True
            calib_clicks = []
        elif key == ord('r') and not calibration_mode:
            trigger_color_pick("Red")
        elif key == ord('g') and not calibration_mode:
            trigger_color_pick("Green")
        elif key == ord('b') and not calibration_mode:
            trigger_color_pick("Blue")
        elif key == ord('y') and not calibration_mode:
            trigger_color_pick("Yellow")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
