import cv2
import numpy as np
import time

# ==============================================================================
# 1. 9 FIXED PHYSICAL ROBOT COORDINATES (DERIVED FROM YOUR 4 CORNERS)
# ==============================================================================
# Your taught 4 physical corner poses [X, Y, Z, R]
CORNER_TL = np.array([275.630,  35.467, -34.790,  7.332], dtype=np.float32)
CORNER_TR = np.array([275.684, -35.086, -33.627, -7.253], dtype=np.float32)
CORNER_BL = np.array([204.634,  36.592, -35.372, 10.138], dtype=np.float32)
CORNER_BR = np.array([205.347, -33.692, -32.730, -9.318], dtype=np.float32)

def generate_9_physical_slots():
    """Generates a 3x3 (9-slot) array of physical robot positions using bilinear interpolation."""
    slots = []
    for r in range(3):      # Rows: 0=Top, 1=Middle, 2=Bottom
        u = r / 2.0
        for c in range(3):  # Cols: 0=Left, 1=Center, 2=Right
            v = c / 2.0
            pos = (1 - u) * ((1 - v) * CORNER_TL + v * CORNER_TR) + \
                  u * ((1 - v) * CORNER_BL + v * CORNER_BR)
            slots.append(pos)
    return slots

PHYSICAL_SLOTS = generate_9_physical_slots()

# Stacking and Safety Poses
BEFORE_PLACE_POSE = (200.0, 0.00012, 150.0, 0.0)
SAFE_HOVER_Z = 50.0

# Stacking destination heights (Middle stack zone)
CENTER_SLOT_POS = PHYSICAL_SLOTS[4]  # Center slot (Slot 4)
MIDDLE_STACK_POSITIONS = [
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1], -33.637, 0.0), # Level 1
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  -9.650, 0.0), # Level 2
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  15.935, 0.0), # Level 3
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  40.613, 0.0)  # Level 4
]

# Camera Hardware Settings
CAMERA_INDEX = 1
CAMERA_FLIP_MODE = -1

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

# State Variables
calibration_mode = True
calib_clicks = []
camera_grid_corners = None
slot_color_state = ["Empty"] * 9
selected_slot_index = None


# ==============================================================================
# 2. CAMERA GRID CALIBRATION & COLOR DETECTION (9 SLOTS)
# ==============================================================================
def compute_camera_grid_polygons(clicks):
    """Computes bounding quad polygons for each of the 9 visual slots on the camera view."""
    tl, tr, bl, br = [np.array(p, dtype=np.float32) for p in clicks]
    polygons = []

    for r in range(3):
        u0, u1 = r / 3.0, (r + 1) / 3.0
        for c in range(3):
            v0, v1 = c / 3.0, (c + 1) / 3.0

            # 4 vertices of grid cell (r, c)
            p0 = (1 - u0) * ((1 - v0) * tl + v0 * tr) + u0 * ((1 - v0) * bl + v0 * br)
            p1 = (1 - u0) * ((1 - v1) * tl + v1 * tr) + u0 * ((1 - v1) * bl + v1 * br)
            p2 = (1 - u1) * ((1 - v1) * tl + v1 * tr) + u1 * ((1 - v1) * bl + v1 * br)
            p3 = (1 - u1) * ((1 - v0) * tl + v0 * tr) + u1 * ((1 - v0) * bl + v0 * br)

            cell_poly = np.array([p0, p1, p2, p3], dtype=np.int32)
            polygons.append(cell_poly)

    return polygons


def classify_slot_colors(frame, grid_polygons, pixel_threshold=300):
    """Inspects frame inside each of the 9 visual slots to assign colors."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    states = ["Empty"] * 9

    for idx, poly in enumerate(grid_polygons):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)

        best_color = "Empty"
        max_count = 0

        for color_name, config in COLOR_RANGES.items():
            c_mask = cv2.inRange(hsv, config["lower1"], config["upper1"])
            if "lower2" in config:
                c_mask2 = cv2.inRange(hsv, config["lower2"], config["upper2"])
                c_mask = cv2.bitwise_or(c_mask, c_mask2)

            # Restrict color check strictly inside this slot ROI
            slot_c_mask = cv2.bitwise_and(c_mask, c_mask, mask=mask)
            count = cv2.countNonZero(slot_c_mask)

            if count > pixel_threshold and count > max_count:
                max_count = count
                best_color = color_name

        states[idx] = best_color

    return states


# ==============================================================================
# 3. MOUSE CALLBACK & HOTKEYS
# ==============================================================================
def on_mouse_click(event, x, y, flags, param):
    global calibration_mode, calib_clicks, camera_grid_corners, selected_slot_index, slot_color_state

    if event == cv2.EVENT_LBUTTONDOWN:
        # Step 1: User Calibrates 4 Corners of Camera Grid
        if calibration_mode:
            calib_clicks.append([x, y])
            labels = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
            print(f"[CALIB] Corner {len(calib_clicks)}/4 ({labels[len(calib_clicks)-1]}) clicked at ({x}, {y})")

            if len(calib_clicks) == 4:
                camera_grid_corners = compute_camera_grid_polygons(calib_clicks)
                calibration_mode = False
                print("[SUCCESS] Camera aligned to 9-Slot Grid system!")
            return

        # Step 2: User clicks a slot on screen
        if camera_grid_corners is not None:
            for idx, poly in enumerate(camera_grid_corners):
                if cv2.pointPolygonTest(poly, (x, y), False) >= 0:
                    color = slot_color_state[idx]
                    if color != "Empty":
                        selected_slot_index = idx
                        print(f"\n[CLICKED SLOT {idx+1}] Contains: {color} Block")
                    else:
                        print(f"\n[CLICKED SLOT {idx+1}] Slot is Empty!")
                    return


def trigger_pick_by_color(target_color):
    global selected_slot_index, slot_color_state
    for idx, color in enumerate(slot_color_state):
        if color.lower() == target_color.lower():
            selected_slot_index = idx
            print(f"\n[HOTKEY TRIGGERED] Found {target_color} at Slot {idx+1}")
            return
    print(f"[WARN] No {target_color} block detected in any slot!")


# ==============================================================================
# 4. ROBOT CONTROLLER
# ==============================================================================
def pump_camera(cap, duration=1.0):
    end_time = time.time() + duration
    while time.time() < end_time:
        ret, frame = cap.read()
        if ret and frame is not None:
            if CAMERA_FLIP_MODE is not None:
                frame = cv2.flip(frame, CAMERA_FLIP_MODE)
            cv2.imshow("Camera View", frame)
            cv2.waitKey(1)


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

    def move_to(self, x, y, z, r, cap=None):
        if self.device is None:
            print(f"[OFFLINE] Move to X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}, R:{r:.2f}")
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

    def pick_from_slot_and_place(self, slot_idx, stack_level_index, cap=None):
        # Retrieve preset physical coordinates directly from array
        pick_x, pick_y, pick_z, pick_r = PHYSICAL_SLOTS[slot_idx]
        place_x, place_y, place_z, place_r = MIDDLE_STACK_POSITIONS[stack_level_index]

        print(f" -> Executing Pick from Slot {slot_idx+1} Pos: ({pick_x:.2f}, {pick_y:.2f}, {pick_z:.2f})")

        # 1. Hover above selected physical slot
        self.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)
        
        # 2. Lower to fixed slot pick height and suck
        self.move_to(pick_x, pick_y, pick_z, pick_r, cap)
        self.set_suck(True)
        
        # 3. Retract
        self.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)
        
        # 4. Waypoint
        self.move_to(*BEFORE_PLACE_POSE, cap=cap)
        
        # 5. Place on stack
        self.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
        self.move_to(place_x, place_y, place_z, place_r, cap)
        self.set_suck(False)
        
        # 6. Return Home
        self.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
        self.move_to(*BEFORE_PLACE_POSE, cap=cap)


# ==============================================================================
# 5. MAIN LOOP
# ==============================================================================
def main():
    global calibration_mode, calib_clicks, camera_grid_corners, slot_color_state, selected_slot_index

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)

    robot = RobotController(port="/dev/ttyUSB0")
    stacked_count = 0
    MAX_STACK = len(MIDDLE_STACK_POSITIONS)

    cv2.namedWindow("Camera View")
    cv2.setMouseCallback("Camera View", on_mouse_click)

    print("\n=======================================================")
    print("DISCRETE 9-SLOT GRID CONTROL SYSTEM")
    print(" - Step 1: Click the 4 corners of the grid overlay on screen:")
    print("   1. Top-Left  2. Top-Right  3. Bottom-Left  4. Bottom-Right")
    print(" - Step 2: Camera will detect block colors in 9 grid slots.")
    print(" - Step 3: Click a slot or press [r, g, b, y] to pick.")
    print(" - Press 'c' to recalibrate camera grid alignment.")
    print("=======================================================\n")

    calib_labels = ["1. Top-Left", "2. Top-Right", "3. Bottom-Left", "4. Bottom-Right"]

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        if CAMERA_FLIP_MODE is not None:
            frame = cv2.flip(frame, CAMERA_FLIP_MODE)

        display_frame = frame.copy()

        # MODE A: CAMERA GRID CALIBRATION
        if calibration_mode:
            step = len(calib_clicks)
            if step < 4:
                cv2.putText(display_frame, f"CALIB: Click Grid Corner {calib_labels[step]}",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            for idx, pt in enumerate(calib_clicks):
                cv2.circle(display_frame, tuple(pt), 6, (0, 255, 255), -1)
                cv2.putText(display_frame, calib_labels[idx], (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # MODE B: OPERATIONAL 9-SLOT SCANNING
        else:
            slot_color_state = classify_slot_colors(frame, camera_grid_corners)

            cv2.putText(display_frame, f"Stacked: {stacked_count}/{MAX_STACK} | Click Slot or Press [r,g,b,y]",
                        (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # Draw 9 Slots & Recognized Colors
            for idx, poly in enumerate(camera_grid_corners):
                color_name = slot_color_state[idx]
                bgr = COLOR_RANGES[color_name]["bgr"] if color_name in COLOR_RANGES else (128, 128, 128)

                # Draw slot bounding outline
                cv2.polylines(display_frame, [poly], isClosed=True, color=bgr, thickness=2)

                # Calculate center of polygon for text label
                M = cv2.moments(poly)
                if M["m00"] != 0:
                    mcx = int(M["m10"] / M["m00"])
                    mcy = int(M["m01"] / M["m00"])
                    
                    # Display Slot Number and Detected Color
                    label = f"S{idx+1}: {color_name}"
                    cv2.putText(display_frame, label, (mcx - 35, mcy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # EXECUTE PICK FROM DISCRETE SLOT
        if selected_slot_index is not None and not calibration_mode:
            if stacked_count < MAX_STACK:
                target_slot = selected_slot_index
                color_picked = slot_color_state[target_slot]

                print(f"\n[ACTION] Dispatching Arm to Slot {target_slot+1} ({color_picked})")
                robot.pick_from_slot_and_place(target_slot, stacked_count, cap=cap)
                stacked_count += 1
            else:
                print("[INFO] Stack height limit reached!")

            selected_slot_index = None

        cv2.imshow("Camera View", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            print("\n[RE-CALIBRATING CAMERA GRID] Click 4 workspace corners in order.")
            calibration_mode = True
            calib_clicks = []
        elif key == ord('r') and not calibration_mode:
            trigger_pick_by_color("Red")
        elif key == ord('g') and not calibration_mode:
            trigger_pick_by_color("Green")
        elif key == ord('b') and not calibration_mode:
            trigger_pick_by_color("Blue")
        elif key == ord('y') and not calibration_mode:
            trigger_pick_by_color("Yellow")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
