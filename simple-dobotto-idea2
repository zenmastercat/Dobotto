import cv2
import numpy as np
import time

# ==============================================================================
# 1. TAUGHT PHYSICAL CORNERS & 3x3 XO GRID GENERATION
# ==============================================================================
# Taught corner positions from your robot arm: [X, Y, Z, R]
CORNER_TL = np.array([275.630,  35.467, -34.790,  7.332], dtype=np.float32)  # Slot 1
CORNER_TR = np.array([275.684, -35.086, -33.627, -7.253], dtype=np.float32)  # Slot 3
CORNER_BL = np.array([204.634,  36.592, -35.372, 10.138], dtype=np.float32)  # Slot 7
CORNER_BR = np.array([205.347, -33.692, -32.730, -9.318], dtype=np.float32)  # Slot 9

def generate_xo_physical_slots():
    """
    Interpolates a 3x3 XO grid (9 slots):
    [ Slot 1 | Slot 2 | Slot 3 ]
    [ Slot 4 | Slot 5 | Slot 6 ]
    [ Slot 7 | Slot 8 | Slot 9 ]
    """
    slots = []
    for r in range(3):      # Row 0 (Top), Row 1 (Mid), Row 2 (Bot)
        u = r / 2.0
        for c in range(3):  # Col 0 (Left), Col 1 (Center), Col 2 (Right)
            v = c / 2.0
            pos = (1 - u) * ((1 - v) * CORNER_TL + v * CORNER_TR) + \
                  u * ((1 - v) * CORNER_BL + v * CORNER_BR)
            slots.append(pos)
    return slots

# 9 Physical Robot Coordinates
XO_PHYSICAL_SLOTS = generate_xo_physical_slots()

# Placement / Drop Zone (e.g. Off-board stack or designated box)
BEFORE_PLACE_POSE = (200.0, 0.00012, 150.0, 0.0)
SAFE_HOVER_Z = 50.0

CENTER_SLOT_POS = XO_PHYSICAL_SLOTS[4]  # Center XO cell (Slot 5)
STACK_TARGET_POSITIONS = [
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1], -33.637, 0.0), # Level 1
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  -9.650, 0.0), # Level 2
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  15.935, 0.0), # Level 3
    (CENTER_SLOT_POS[0], CENTER_SLOT_POS[1],  40.613, 0.0)  # Level 4
]

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

calibration_mode = True
calib_clicks = []
xo_grid_polygons = None
board_state = ["Empty"] * 9
selected_slot_index = None


# ==============================================================================
# 2. VISION: XO BOARD CALIBRATION & COLOR SCANNING
# ==============================================================================
def create_xo_grid_polygons(clicks):
    """Builds the 3x3 visual bounding boxes from 4 user clicks."""
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
    """Scans all 9 XO slots and identifies which color block occupies each cell."""
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
        # Step 1: Calibration (User clicks 4 outer corners of XO board)
        if calibration_mode:
            calib_clicks.append([x, y])
            labels = ["Top-Left (Slot 1)", "Top-Right (Slot 3)", "Bottom-Left (Slot 7)", "Bottom-Right (Slot 9)"]
            print(f"[CALIB] Corner {len(calib_clicks)}/4 ({labels[len(calib_clicks)-1]}) set at ({x}, {y})")

            if len(calib_clicks) == 4:
                xo_grid_polygons = create_xo_grid_polygons(calib_clicks)
                calibration_mode = False
                print("[READY] XO Board calibrated successfully!")
            return

        # Step 2: User clicks any XO cell to pick its block
        if xo_grid_polygons is not None:
            for idx, poly in enumerate(xo_grid_polygons):
                if cv2.pointPolygonTest(poly, (x, y), False) >= 0:
                    color = board_state[idx]
                    if color != "Empty":
                        selected_slot_index = idx
                        print(f"\n[CLICKED SLOT {idx+1}] Selected {color} Block")
                    else:
                        print(f"\n[CLICKED SLOT {idx+1}] Cell is Empty")
                    return


def trigger_color_pick(target_color):
    global selected_slot_index, board_state
    for idx, color in enumerate(board_state):
        if color.lower() == target_color.lower():
            selected_slot_index = idx
            print(f"\n[HOTKEY] Found {target_color} block at Slot {idx+1}")
            return
    print(f"[WARN] No {target_color} block on the XO board!")


# ==============================================================================
# 4. ROBOT EXECUTION
# ==============================================================================
def pump_camera(cap, duration=1.0):
    end_time = time.time() + duration
    while time.time() < end_time:
        ret, frame = cap.read()
        if ret and frame is not None:
            if CAMERA_FLIP_MODE is not None:
                frame = cv2.flip(frame, CAMERA_FLIP_MODE)
            cv2.imshow("XO Board View", frame)
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

    def pick_from_xo_slot(self, slot_idx, stack_level_index, cap=None):
        # Pick directly from predefined 3D slot coordinate
        pick_x, pick_y, pick_z, pick_r = XO_PHYSICAL_SLOTS[slot_idx]
        place_x, place_y, place_z, place_r = STACK_TARGET_POSITIONS[stack_level_index]

        print(f" -> Picking from XO Slot {slot_idx+1} | Pos: ({pick_x:.2f}, {pick_y:.2f}, {pick_z:.2f})")

        # Approach, Pick, Lift, Move to Stack, Drop
        self.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)
        self.move_to(pick_x, pick_y, pick_z, pick_r, cap)
        self.set_suck(True)
        self.move_to(pick_x, pick_y, SAFE_HOVER_Z, pick_r, cap)
        
        self.move_to(*BEFORE_PLACE_POSE, cap=cap)
        self.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
        self.move_to(place_x, place_y, place_z, place_r, cap)
        self.set_suck(False)
        self.move_to(place_x, place_y, SAFE_HOVER_Z, place_r, cap)
        self.move_to(*BEFORE_PLACE_POSE, cap=cap)


# ==============================================================================
# 5. MAIN LOOP
# ==============================================================================
def main():
    global calibration_mode, calib_clicks, xo_grid_polygons, board_state, selected_slot_index

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)

    robot = RobotController(port="/dev/ttyUSB0")
    stacked_count = 0
    MAX_STACK = len(STACK_TARGET_POSITIONS)

    cv2.namedWindow("XO Board View")
    cv2.setMouseCallback("XO Board View", on_mouse_click)

    print("\n=======================================================")
    print("3x3 XO TABLE CONTROL SYSTEM")
    print(" - Align camera: Click 4 corners on screen in order:")
    print("   1. Slot 1 (Top-Left)     2. Slot 3 (Top-Right)")
    print("   3. Slot 7 (Bottom-Left)  4. Slot 9 (Bottom-Right)")
    print(" - Click any slot cell or press [r, g, b, y] to pick.")
    print(" - Press 'c' to realign the camera grid.")
    print("=======================================================\n")

    calib_labels = ["1. Slot 1 (TL)", "2. Slot 3 (TR)", "3. Slot 7 (BL)", "4. Slot 9 (BR)"]

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        if CAMERA_FLIP_MODE is not None:
            frame = cv2.flip(frame, CAMERA_FLIP_MODE)

        display_frame = frame.copy()

        # STATE A: CAMERA CALIBRATION
        if calibration_mode:
            step = len(calib_clicks)
            if step < 4:
                cv2.putText(display_frame, f"CALIBRATION: Click {calib_labels[step]}",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            for idx, pt in enumerate(calib_clicks):
                cv2.circle(display_frame, tuple(pt), 6, (0, 255, 255), -1)
                cv2.putText(display_frame, calib_labels[idx], (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # STATE B: XO BOARD OVERLAY & COLOR DETECTION
        else:
            board_state = scan_xo_board_colors(frame, xo_grid_polygons)

            cv2.putText(display_frame, f"Stacked: {stacked_count}/{MAX_STACK} | Click XO Cell or [r,g,b,y]",
                        (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # Draw XO Board cells
            for idx, poly in enumerate(xo_grid_polygons):
                color_name = board_state[idx]
                bgr = COLOR_RANGES[color_name]["bgr"] if color_name in COLOR_RANGES else (80, 80, 80)

                # Cell box outline
                cv2.polylines(display_frame, [poly], isClosed=True, color=bgr, thickness=2)

                # Center label
                M = cv2.moments(poly)
                if M["m00"] != 0:
                    mcx = int(M["m10"] / M["m00"])
                    mcy = int(M["m01"] / M["m00"])
                    cv2.putText(display_frame, f"[{idx+1}] {color_name}", (mcx - 30, mcy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # EXECUTE PICK FROM XO CELL
        if selected_slot_index is not None and not calibration_mode:
            if stacked_count < MAX_STACK:
                slot_num = selected_slot_index
                color = board_state[slot_num]

                print(f"\n[ACTION] Picking {color} block from XO Cell {slot_num + 1}")
                robot.pick_from_xo_slot(slot_num, stacked_count, cap=cap)
                stacked_count += 1
            else:
                print("[INFO] Stack capacity reached!")

            selected_slot_index = None

        cv2.imshow("XO Board View", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            print("\n[CALIBRATION] Click 4 corners of XO table on camera window.")
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
