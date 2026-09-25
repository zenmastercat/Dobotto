import cv2
import numpy as np
import time
import math
import sys

# ==============================================================================
# GLOBAL CONFIGURATION & DYNAMIC OFFSETS
# ==============================================================================
GLOBAL_Z_OFFSET = 0.0          # Tunable on-the-fly via '[' and ']'
BLOCK_HEIGHT = 24.0
TOUCH_DESCENT_OFFSET = -1.5    # Slight over-travel for solid vacuum seal
SAFE_AIR_CLEARANCE = 50.0      # Z-height for safe planar movement

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

# State Tracking
calibration_mode = True
calib_clicks = []
xo_grid_polygons = None
board_state = ["Empty"] * 9
selected_slot_index = None
initial_pipe_angle = 0.0


# ==============================================================================
# 1. HARDWARE CONTROLLER (WITH QUEUE, ALARMS & REACH LIMITS)
# ==============================================================================
class RobotController:
    def __init__(self, port="/dev/ttyUSB0"):
        self.device = None
        try:
            from pydobot import Dobot
            self.device = Dobot(port=port)
            print(f"[INFO] Dobot successfully connected on {port}.")
            
            self.clear_alarms()
            self.start_motion_queue()
            self.set_speed(120, 120)
            
        except Exception as e:
            print(f"[WARN] Dobot hardware offline (Simulation mode): {e}")

    def clear_alarms(self):
        if self.device is None: return
        try:
            if hasattr(self.device, 'clear_alarms_state'):
                self.device.clear_alarms_state()
            if hasattr(self.device, '_send_command'):
                from pydobot.message import Message
                msg = Message()
                msg.id = 20; msg.rw = 1; msg.is_queued = 0
                self.device._send_command(msg)
            time.sleep(0.05)
        except: pass

    def start_motion_queue(self):
        if self.device is None: return
        try:
            if hasattr(self.device, '_send_command'):
                from pydobot.message import Message
                msg = Message()
                msg.id = 240; msg.rw = 1; msg.is_queued = 0
                self.device._send_command(msg)
            time.sleep(0.05)
        except: pass

    def set_speed(self, velocity, acceleration):
        if self.device is None: return
        try:
            self.device.speed(velocity, acceleration)
        except TypeError:
            self.device.speed(velocity)

    def get_current_pose(self):
        """Reads current [X, Y, Z, R] directly from the Dobot arm safely across pydobot library variants."""
        if self.device is None:
            return np.array([240.0, 0.0, -34.0, 0.0], dtype=np.float32)

        if hasattr(self.device, 'get_pose'):
            pose_obj = self.device.get_pose()
        elif hasattr(self.device, 'pose') and callable(getattr(self.device, 'pose')):
            pose_obj = self.device.pose()
        elif hasattr(self.device, 'pose'):
            pose_obj = self.device.pose
        else:
            raise AttributeError("Installed Dobot library has neither 'get_pose()' nor 'pose()' method.")

        if hasattr(pose_obj, 'position'):
            pos = pose_obj.position
            return np.array([float(pos.x), float(pos.y), float(pos.z), float(pos.r)], dtype=np.float32)
        if hasattr(pose_obj, 'x') and hasattr(pose_obj, 'y'):
            return np.array([float(pose_obj.x), float(pose_obj.y), float(pose_obj.z), float(getattr(pose_obj, 'r', 0.0))], dtype=np.float32)
        if isinstance(pose_obj, (tuple, list, np.ndarray)):
            return np.array([float(pose_obj[0]), float(pose_obj[1]), float(pose_obj[2]), float(pose_obj[3])], dtype=np.float32)

        raise ValueError(f"Unrecognized pose format returned: {type(pose_obj)}")

    def move_to(self, x, y, z, r):
        """Executes motion with boundary checks and mechanical wait=True sync."""
        dist_2d = math.sqrt(x**2 + y**2)
        if dist_2d < 120.0 or dist_2d > 350.0:
            print(f"[REJECT] Target (X:{x:.1f}, Y:{y:.1f}) out of kinematic reach ({dist_2d:.1f}mm)!")
            return False

        z = max(-75.0, min(z, 120.0))
        r = max(-135.0, min(r, 135.0))

        if self.device is None:
            print(f"[OFFLINE MOVE] X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}, R:{r:.1f}")
            time.sleep(0.5)
            return True

        self.clear_alarms()
        try:
            # Force wait=True so commands complete fully before next step prevents mid-air truncation
            self.device.move_to(x=x, y=y, z=z, r=r, wait=True)
            return True
        except Exception as e:
            print(f"[FAIL] Motion execution error: {e}")
            return False

    def set_suck(self, enable=True):
        if self.device is None: return
        self.device.suck(enable)
        time.sleep(0.4 if enable else 0.2)


# ==============================================================================
# 2. SPATIAL LOGIC & TEACHING INTERFACES
# ==============================================================================
def select_initial_pipe_orientation():
    print("\n" + "="*60)
    print(" INITIAL PIPE / END-EFFECTOR ORIENTATION SETUP")
    print("   [1] Upfront  (0 degrees)")
    print("   [2] Right    (+90 degrees)")
    print("   [3] Back     (180 degrees)")
    print("   [4] Left     (-90 degrees)")
    print("="*60)

    angle_map = {"1": 0.0, "2": 90.0, "3": 180.0, "4": -90.0}
    while True:
        choice = input("Enter choice [1-4] (Default is 1): ").strip()
        if choice in angle_map: return angle_map[choice]
        if choice == "": return 0.0

def calculate_slot_target_rotation(slot_idx, base_angle):
    """Calculates relative rotation per grid position to prevent umbilical wrapping."""
    if slot_idx in [0, 3, 6]:   offset = 90.0    # Left side slots
    elif slot_idx in [2, 5, 8]: offset = -90.0   # Right side slots
    elif slot_idx == 7:         offset = 180.0   # Back slot
    else:                       offset = 0.0     # Center column

    final_r = base_angle + offset
    while final_r > 180.0: final_r -= 360.0
    while final_r < -180.0: final_r += 360.0
    return max(-135.0, min(final_r, 135.0))

def teach_robot_corners(robot):
    print("\n" + "="*60)
    print(" STEP 1: PHYSICAL ROBOT ARM CORNER CALIBRATION")
    print(" Lower nozzle tip DIRECTLY ON TOP OF BLOCK SURFACE at each corner.")
    print("="*60)

    corners = ["Slot 1 (Top-Left)", "Slot 3 (Top-Right)", "Slot 7 (Bot-Left)", "Slot 9 (Bot-Right)"]
    taught = []

    for name in corners:
        print(f"\n[TEACHING] Move arm to [{name}]")
        print(" -> Monitoring live serial stream... Press ENTER when positioned.")
        
        last_pose = robot.get_current_pose()
        try:
            if sys.platform != "win32":
                import select
                while True:
                    last_pose = robot.get_current_pose()
                    print(f"\r   LIVE POS: X:{last_pose[0]:6.1f} | Y:{last_pose[1]:6.1f} | Z:{last_pose[2]:6.1f}  [Press ENTER]", end="", flush=True)
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        sys.stdin.readline()
                        break
            else:
                import msvcrt
                while True:
                    last_pose = robot.get_current_pose()
                    print(f"\r   LIVE POS: X:{last_pose[0]:6.1f} | Y:{last_pose[1]:6.1f} | Z:{last_pose[2]:6.1f}  [Press ENTER]", end="", flush=True)
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        if key in [b'\r', b'\n']: break
                    time.sleep(0.1)
        except Exception:
            input(f"\n>> Press ENTER when positioned at [{name}]...")
            last_pose = robot.get_current_pose()

        taught.append(last_pose)
        print(f"\n   [SAVED] {name} -> X:{last_pose[0]:.1f}, Y:{last_pose[1]:.1f}, Z:{last_pose[2]:.1f}")
    return taught

def generate_xo_physical_slots(corners):
    tl, tr, bl, br = corners
    slots = []
    for r in range(3):
        u = r / 2.0
        for c in range(3):
            v = c / 2.0
            pos = (1 - u) * ((1 - v) * tl + v * tr) + u * ((1 - v) * bl + v * br)
            slots.append(pos)
    return slots


# ==============================================================================
# 3. VISION & UI EVENT HANDLERS
# ==============================================================================
def create_xo_grid_polygons(clicks):
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
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    states = ["Empty"] * 9
    for idx, poly in enumerate(grid_polygons):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)
        highest_count = 0
        detected = "Empty"
        for name, config in COLOR_RANGES.items():
            c_mask = cv2.inRange(hsv, config["lower1"], config["upper1"])
            if "lower2" in config:
                c_mask = cv2.bitwise_or(c_mask, cv2.inRange(hsv, config["lower2"], config["upper2"]))
            cell_mask = cv2.bitwise_and(c_mask, c_mask, mask=mask)
            count = cv2.countNonZero(cell_mask)
            if count > min_pixels and count > highest_count:
                highest_count = count
                detected = name
        states[idx] = detected
    return states

def on_mouse_click(event, x, y, flags, param):
    global calibration_mode, calib_clicks, xo_grid_polygons, selected_slot_index, board_state
    if event == cv2.EVENT_LBUTTONDOWN:
        if calibration_mode:
            calib_clicks.append([x, y])
            labels = ["Slot 1 (TL)", "Slot 3 (TR)", "Slot 7 (BL)", "Slot 9 (BR)"]
            print(f"[CAM CALIB] Corner {len(calib_clicks)}/4 ({labels[len(calib_clicks)-1]}) set.")
            if len(calib_clicks) == 4:
                xo_grid_polygons = create_xo_grid_polygons(calib_clicks)
                calibration_mode = False
                print("[READY] Camera aligned to physical XO table!")
        elif xo_grid_polygons is not None:
            for idx, poly in enumerate(xo_grid_polygons):
                if cv2.pointPolygonTest(poly, (x, y), False) >= 0:
                    if board_state[idx] != "Empty":
                        selected_slot_index = idx
                    return

def trigger_color_pick(target_color):
    global selected_slot_index, board_state
    for idx, color in enumerate(board_state):
        if color.lower() == target_color.lower():
            selected_slot_index = idx
            return
    print(f"[WARN] No {target_color} block detected!")


# ==============================================================================
# 4. MAIN PROGRAM FLOW
# ==============================================================================
def main():
    global calibration_mode, calib_clicks, xo_grid_polygons, board_state, selected_slot_index, GLOBAL_Z_OFFSET

    initial_pipe_angle = select_initial_pipe_orientation()
    robot = RobotController(port="/dev/ttyUSB0")

    raw_corners = teach_robot_corners(robot)
    physical_slots = generate_xo_physical_slots(raw_corners)
    center_cell_pos = physical_slots[4]  # Slot 5 is the mathematical center

    stack_target_positions = [
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + (0 * BLOCK_HEIGHT), 0.0),
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + (1 * BLOCK_HEIGHT), 0.0),
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + (2 * BLOCK_HEIGHT), 0.0),
        (center_cell_pos[0], center_cell_pos[1], center_cell_pos[2] + (3 * BLOCK_HEIGHT), 0.0)
    ]

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
        if not ret or frame is None: continue
        if CAMERA_FLIP_MODE is not None: frame = cv2.flip(frame, CAMERA_FLIP_MODE)

        display_frame = frame.copy()

        if calibration_mode:
            step = len(calib_clicks)
            if step < 4:
                cv2.putText(display_frame, f"CLICK CAMERA: {calib_labels[step]}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            for idx, pt in enumerate(calib_clicks):
                cv2.circle(display_frame, tuple(pt), 6, (0, 255, 255), -1)
                cv2.putText(display_frame, calib_labels[idx], (pt[0] + 10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        else:
            board_state = scan_xo_board_colors(frame, xo_grid_polygons)
            
            # Status HUD
            cv2.putText(display_frame, f"Stacked: {stacked_count}/{MAX_STACK} | Z-Tune: {GLOBAL_Z_OFFSET:+.1f}mm", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(display_frame, "Keys: [=Lower Z | ]=Raise Z | c=Recalib | r,g,b,y=Pick Color", (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            for idx, poly in enumerate(xo_grid_polygons):
                color_name = board_state[idx]
                bgr = COLOR_RANGES[color_name]["bgr"] if color_name in COLOR_RANGES else (80, 80, 80)
                cv2.polylines(display_frame, [poly], isClosed=True, color=bgr, thickness=2)

                M = cv2.moments(poly)
                if M["m00"] != 0:
                    mcx, mcy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                    cv2.putText(display_frame, f"[{idx+1}] {color_name}", (mcx - 30, mcy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # SYNCHRONIZED EXECUTION ENGINE
        if selected_slot_index is not None and not calibration_mode:
            if stacked_count < MAX_STACK:
                slot_idx = selected_slot_index
                color = board_state[slot_idx]

                # 1. Fetch Spatial Data
                pick_x, pick_y, surface_z, _ = physical_slots[slot_idx]
                place_x, place_y, base_place_z, _ = stack_target_positions[stacked_count]

                # 2. Apply Dynamic Tooling Adjustments
                actual_pick_z = surface_z + TOUCH_DESCENT_OFFSET + GLOBAL_Z_OFFSET
                actual_place_z = base_place_z + TOUCH_DESCENT_OFFSET + GLOBAL_Z_OFFSET
                
                # 3. Orient end-effector based on slot location
                target_r = calculate_slot_target_rotation(slot_idx, initial_pipe_angle)
                
                # 4. Safe Z-lift heights based on stack level
                dynamic_lift_z = max(surface_z, base_place_z) + SAFE_AIR_CLEARANCE + (stacked_count * BLOCK_HEIGHT)

                print(f"\n[ACTION] Picking {color} Block from Cell {slot_idx+1}")
                
                # Freeze camera frame slightly before blocking motion begins
                cv2.imshow("XO Board View", display_frame)
                cv2.waitKey(10)

                # Blocking Motion Sequence (Safe Mid-air transition)
                robot.move_to(pick_x, pick_y, dynamic_lift_z, target_r)
                robot.move_to(pick_x, pick_y, actual_pick_z, target_r)
                robot.set_suck(True)
                robot.move_to(pick_x, pick_y, dynamic_lift_z, target_r)

                robot.move_to(place_x, place_y, dynamic_lift_z, initial_pipe_angle)
                robot.move_to(place_x, place_y, actual_place_z, initial_pipe_angle)
                robot.set_suck(False)
                robot.move_to(place_x, place_y, dynamic_lift_z, initial_pipe_angle)

                stacked_count += 1
            else:
                print("[INFO] Maximum stack limit reached! Clear area to continue.")
                
            selected_slot_index = None

        cv2.imshow("XO Board View", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('['):
            GLOBAL_Z_OFFSET -= 2.0
            print(f"[Z-TUNE] Lowered tool height by 2mm (Offset: {GLOBAL_Z_OFFSET:+.1f}mm)")
        elif key == ord(']'):
            GLOBAL_Z_OFFSET += 2.0
            print(f"[Z-TUNE] Raised tool height by 2mm (Offset: {GLOBAL_Z_OFFSET:+.1f}mm)")
        elif key == ord('c'):
            calibration_mode = True
            calib_clicks = []
        elif key == ord('r') and not calibration_mode: trigger_color_pick("Red")
        elif key == ord('g') and not calibration_mode: trigger_color_pick("Green")
        elif key == ord('b') and not calibration_mode: trigger_color_pick("Blue")
        elif key == ord('y') and not calibration_mode: trigger_color_pick("Yellow")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
