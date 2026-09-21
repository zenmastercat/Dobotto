# # import cv2
# # import numpy as np
# # import pydobot
# # from serial.tools import list_ports
# # import time
# import cv2
# import glob
# import numpy as np
# import pydobot
# from serial.tools import list_ports
# import time

# # ==========================================
# # 1. CONFIGURATION & CALIBRATION
# # ==========================================

# # Define HSV color boundaries for the blocks (Adjust these based on your room lighting)
# COLOR_RANGES = {
#     'red':   [(0, 120, 70), (10, 255, 255)], # Red can also wrap around to 170-180 in OpenCV
#     'green': [(40, 50, 50), (80, 255, 255)],
#     'blue':  [(100, 150, 0), (140, 255, 255)],
#     'yellow':[(20, 100, 100), (30, 255, 255)]
# }

# # Stacking Base Coordinates (Where the tower will be built)
# STACK_X = 200.0
# STACK_Y = 0.0
# BASE_Z = -40.0       # Z-height of the table/surface
# BLOCK_HEIGHT = 25.0  # Height of a single block in mm
# SAFE_Z = 50.0        # Safe height to travel without hitting anything

# # ==========================================
# # 2. HELPER FUNCTIONS
# # ==========================================

# def pixels_to_robot_coords(px, py):
#     """
#     Translates camera pixel coordinates (x, y) to Dobot physical coordinates (X, Y).
#     NOTE: This is a simplified linear mapping. In reality, you will need to map 
#     4 corners of your camera view to 4 known Dobot coordinates using cv2.getPerspectiveTransform.
#     """
#     # PLACEHOLDER CALIBRATION: You must calculate your scale and offsets!
#     scale_x = 0.5  # mm per pixel
#     scale_y = 0.5  # mm per pixel
#     offset_x = 100 # Robot X offset from camera origin
#     offset_y = -50 # Robot Y offset from camera origin
    
#     robot_x = (py * scale_x) + offset_x # Note: Camera Y is often Robot X depending on setup
#     robot_y = (px * scale_y) + offset_y
    
#     return robot_x, robot_y

# def find_block_by_color(frame, color_name):
#     """Finds the center pixel (X, Y) of the largest block of the requested color."""
#     if color_name not in COLOR_RANGES:
#         print(f"Color {color_name} not defined.")
#         return None
        
#     lower, upper = COLOR_RANGES[color_name]
#     lower = np.array(lower, dtype="uint8")
#     upper = np.array(upper, dtype="uint8")
    
#     # Convert to HSV and apply mask
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, lower, upper)
    
#     # Clean up the mask
#     mask = cv2.erode(mask, None, iterations=2)
#     mask = cv2.dilate(mask, None, iterations=2)
    
#     # Find contours
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     if len(contours) > 0:
#         # Find the largest contour (the block)
#         c = max(contours, key=cv2.contourArea)
#         M = cv2.moments(c)
        
#         # Calculate center
#         if M["m00"] != 0:
#             cX = int(M["m10"] / M["m00"])
#             cY = int(M["m01"] / M["m00"])
#             return cX, cY
            
#     return None

# # ==========================================
# # 3. MAIN LOGIC
# # ==========================================

# def get_camera_port():
#     """Scans /dev/video* ports and returns the first functional camera device path or index."""
#     # Find all video device paths in /dev/
#     video_ports = sorted(glob.glob('/dev/video*'))
    
#     for port in video_ports:
#         # Use Video4Linux2 (V4L2) backend for Linux
#         cap = cv2.VideoCapture(port, cv2.CAP_V4L2)
#         if cap.isOpened():
#             ret, frame = cap.read()
#             cap.release()
#             if ret:
#                 return port
                
#     # Fallback to standard index loop if no /dev/video nodes matched
#     for index in range(4):
#         cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
#         if cap.isOpened():
#             ret, frame = cap.read()
#             cap.release()
#             if ret:
#                 return index

#     return None

# def main():
#     # # 1. Connect to Dobot
#     # available_ports = list_ports.comports()
#     # if not available_ports:
#     #     print("No Dobot found. Check USB connection.")
#     #     return
        
#     # dobot_port = available_ports[0].device
#     # print(f"Connecting to Dobot on {dobot_port}...")
#     # device = pydobot.Dobot(port=dobot_port, verbose=False)
#     # Replace the old port detection block with this:
#     available_ports = list_ports.comports()
#     dobot_port = None

#     for p in available_ports:
#         # Filter for standard USB-to-Serial adapters used by Dobot
#         if "USB" in p.device or "ACM" in p.device:
#             dobot_port = p.device
#             break

#     if not dobot_port:
#         print("No Dobot found! Please ensure it is plugged in via USB.")
#         return

#     print(f"Connecting to Dobot on {dobot_port}...")
#     device = pydobot.Dobot(port=dobot_port, verbose=False)
    
#     # 2. Initialize Camera
#     # cap = cv2.VideoCapture(1) # Use 1 or 2 if using an external USB webcam
#     # time.sleep(2) # Camera warmup
#     # 2. Initialize Camera via detected port
#     cam_port = get_camera_port()
#     if cam_port is None:
#         print("No camera found on any /dev/video port!")
#         return

#     print(f"Connecting to camera on {cam_port}...")
#     cap = cv2.VideoCapture(cam_port, cv2.CAP_V4L2)
#     time.sleep(2)  # Camera warmup
    
#     # 3. Get User Input
#     print("\nAvailable colors: red, green, blue, yellow")
#     user_input = input("Enter the order to stack (e.g., red, blue, green): ")
#     stack_order = [color.strip().lower() for color in user_input.split(',')]
    
#     # 4. Execute the Pick and Place sequence
#     for index, color in enumerate(stack_order):
#         print(f"\nLooking for {color} block...")
        
#         # Grab a few frames to clear buffer and get a fresh image
#         for _ in range(5):
#             ret, frame = cap.read()
            
#         block_pixels = find_block_by_color(frame, color)
        
#         if not block_pixels:
#             print(f"Could not find {color} block! Skipping...")
#             continue
            
#         px, py = block_pixels
#         robot_x, robot_y = pixels_to_robot_coords(px, py)
#         print(f"Found {color} at pixels: ({px}, {py}) -> Robot Coords: ({robot_x:.1f}, {robot_y:.1f})")
        
#         # Calculate stacking height (Z increases with each block)
#         current_place_z = BASE_Z + (index * BLOCK_HEIGHT)
        
#         # --- KINEMATICS SEQUENCE ---
#         # Move above the block
#         device.move_to(robot_x, robot_y, SAFE_Z, 0, wait=True)
#         # Drop down to block
#         device.move_to(robot_x, robot_y, BASE_Z, 0, wait=True)
#         # Turn on suction
#         device.suck(True)
#         time.sleep(0.5)
#         # Pull straight up
#         device.move_to(robot_x, robot_y, SAFE_Z, 0, wait=True)
        
#         # Move above stack
#         device.move_to(STACK_X, STACK_Y, SAFE_Z, 0, wait=True)
#         # Drop down to stack height
#         device.move_to(STACK_X, STACK_Y, current_place_z, 0, wait=True)
#         # Turn off suction
#         device.suck(False)
#         time.sleep(0.5)
#         # Pull straight up
#         device.move_to(STACK_X, STACK_Y, SAFE_Z, 0, wait=True)
        
#         print(f"Successfully stacked {color}.")

#     # Cleanup
#     cap.release()
#     device.close()
#     print("\nTask Complete!")

# if __name__ == "__main__":
#     main()

# import os
# # Suppress OpenCV & Qt C++ level warning logs in terminal
# os.environ["OPENCV_LOG_LEVEL"] = "OFF"
# os.environ["QT_LOGGING_RULES"] = "*=false"

# import cv2
# import glob
# import time
# import numpy as np
# import pydobot
# from pydobot.message import Message
# from serial.tools import list_ports

# # ==========================================
# # 1. HARDWARE & ADAPTIVE CONFIGURATION
# # ==========================================

# CAMERA_INDEX_OVERRIDE = None
# DOBOT_PORT_OVERRIDE = None
# TARGET_TOWER_HEIGHT = 4

# # Physical Dobot Arm Heights (in mm)
# SAFE_Z = 25.0        # Safe clearance height above blocks
# BASE_PICK_Z = -45.0  # Default pickup Z height
# BLOCK_HEIGHT = 25.0  # Height of a single block for stacking offset

# # Adaptive Offset Parameters
# ADAPTIVE_PICK_Z_OFFSET = 0.0  
# ADAPTIVE_SUCTION_DWELL = 0.5   
# MISPLACEMENT_COUNT = 0         

# STACK_X = 220.0
# STACK_Y = -100.0

# # ------------------------------------------------------------------
# # Perspective Calibration Points (4 Workspace Corners)
# # ------------------------------------------------------------------
# SRC_PIXELS = np.array([
#     [100, 100],  # Top-Left Corner
#     [540, 100],  # Top-Right Corner
#     [540, 380],  # Bottom-Right Corner
#     [100, 380]   # Bottom-Left Corner
# ], dtype=np.float32)

# DST_ROBOT = np.array([
#     [150.0,  120.0],  # Top-Left Robot (X, Y)
#     [150.0, -120.0],  # Top-Right Robot (X, Y)
#     [320.0, -120.0],  # Bottom-Right Robot (X, Y)
#     [320.0,  120.0]   # Bottom-Left Robot (X, Y)
# ], dtype=np.float32)

# H_MATRIX, _ = cv2.findHomography(SRC_PIXELS, DST_ROBOT)
# H_INVERSE, _ = cv2.findHomography(DST_ROBOT, SRC_PIXELS)

# COLOR_RANGES = {}
# COLOR_BGR = {
#     'red':    (0, 0, 255),
#     'green':  (0, 255, 0),
#     'blue':   (255, 0, 0),
#     'yellow': (0, 255, 255)
# }

# # ==========================================
# # 2. MACHINE LEARNING HOMING CORRECTION MODEL
# # ==========================================

# class HomingCorrectionML:
#     """
#     Online Supervised ML Model (Ridge / Multivariate Regression)
#     Learns mechanical homing hysteresis and joint drift over time.
#     Predicts (dX, dY) corrections for pickup points post-homing.
#     """
#     def __init__(self):
#         self.samples_X = []  # Features: [nominal_x, nominal_y, homing_count]
#         self.samples_Y = []  # Target Offsets: [dx, dy]
#         self.homing_count = 0
#         self.is_trained = False
        
#         try:
#             from sklearn.linear_model import Ridge
#             self.model = Ridge(alpha=0.5)
#             self.use_sklearn = True
#             print("[ML SYSTEM] Scikit-Learn initialized for online regression.")
#         except ImportError:
#             self.use_sklearn = False
#             print("[ML SYSTEM] Scikit-Learn not found. Falling back to NumPy Least-Squares solver.")

#     def register_homing_event(self):
#         """Increments homing counter to track state changes after arm re-homing."""
#         self.homing_count += 1
#         print(f"\n[ML SYSTEM] Homing Event #{self.homing_count} recorded. Updating model context...")

#     def add_observation(self, nominal_x, nominal_y, actual_x, actual_y):
#         """Adds visual alignment error observation to training dataset and updates model."""
#         dx = actual_x - nominal_x
#         dy = actual_y - nominal_y
        
#         self.samples_X.append([nominal_x, nominal_y, float(self.homing_count)])
#         self.samples_Y.append([dx, dy])
        
#         print(f"[ML DATA] Sample recorded: Point ({nominal_x:.1f}, {nominal_y:.1f}) -> Error (dX={dx:.2f}mm, dY={dy:.2f}mm)")
        
#         if len(self.samples_X) >= 3:
#             self.train()

#     def train(self):
#         """Fits the regression model to accumulated homing error samples."""
#         X = np.array(self.samples_X)
#         Y = np.array(self.samples_Y)
#         if self.use_sklearn:
#             self.model.fit(X, Y)
#         self.is_trained = True
#         print(f"[ML SYSTEM] Model retrained on {len(self.samples_X)} physical observations.")

#     def predict(self, nominal_x, nominal_y):
#         """Predicts corrected robot coordinates (X_corr, Y_corr) based on homing history."""
#         if not self.is_trained or len(self.samples_X) < 3:
#             return nominal_x, nominal_y
        
#         feat = np.array([[nominal_x, nominal_y, float(self.homing_count)]])
        
#         if self.use_sklearn:
#             pred_offset = self.model.predict(feat)[0]
#         else:
#             X = np.array(self.samples_X)
#             Y = np.array(self.samples_Y)
#             W, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
#             pred_offset = (feat @ W)[0]

#         corr_x = nominal_x + pred_offset[0]
#         corr_y = nominal_y + pred_offset[1]
#         print(f"[ML PREDICT] Applied ML Homing Correction: dX={pred_offset[0]:.2f}mm, dY={pred_offset[1]:.2f}mm")
#         return corr_x, corr_y

# # Initialize Global ML Model Instance
# ml_corrector = HomingCorrectionML()

# # ==========================================
# # 3. HOMOGRAPHY & VISION FUNCTIONS
# # ==========================================

# def pixels_to_robot_coords(px, py):
#     pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
#     transformed = cv2.perspectiveTransform(pt, H_MATRIX)
#     return float(transformed[0][0][0]), float(transformed[0][0][1])

# def robot_to_pixel_coords(rx, ry):
#     pt = np.array([[[float(rx), float(ry)]]], dtype=np.float32)
#     transformed = cv2.perspectiveTransform(pt, H_INVERSE)
#     return int(round(transformed[0][0][0])), int(round(transformed[0][0][1]))

# def list_system_cameras():
#     print("[SYSTEM] Detecting V4L2 camera devices:")
#     devices = sorted(glob.glob('/sys/class/video4linux/video*'))
#     for dev in devices:
#         try:
#             with open(f"{dev}/name", "r") as f:
#                 name = f.read().strip()
#             idx = int(dev.replace('/sys/class/video4linux/video', ''))
#             print(f"  -> Index {idx}: {name}")
#         except Exception:
#             pass

# def get_working_camera(override_index=None):
#     list_system_cameras()

#     def test_camera_index(idx):
#         print(f"[SYSTEM] Testing camera index {idx}...")
#         cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
#         if not cap.isOpened():
#             return None

#         cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
#         cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#         valid_frame = False
#         for _ in range(25):
#             ret, frame = cap.read()
#             if ret and frame is not None and frame.mean() > 10.0:
#                 valid_frame = True
#                 break
#             time.sleep(0.04)

#         if valid_frame:
#             return cap

#         cap.release()
#         return None

#     if override_index is not None:
#         cap = test_camera_index(override_index)
#         if cap:
#             print(f"[SYSTEM] Successfully attached to camera on index {override_index}")
#             return cap, override_index

#     for idx in [2, 4, 6, 0]:
#         cap = test_camera_index(idx)
#         if cap:
#             print(f"[SYSTEM] Auto-detected active video camera stream on index {idx}")
#             return cap, idx

#     return None, None

# def get_workspace_roi_mask(shape):
#     mask = np.zeros(shape[:2], dtype=np.uint8)
#     pts = SRC_PIXELS.astype(np.int32)
#     cv2.fillPoly(mask, [pts], 255)
#     return mask

# def auto_calibrate_colors(cap):
#     print("[SYSTEM] Auto-calibrating multi-tone color thresholds...")
#     frames = []
#     for _ in range(10):
#         ret, frame = cap.read()
#         if ret and frame is not None:
#             frames.append(frame)
#         time.sleep(0.04)
        
#     if not frames:
#         min_s, min_v = 25, 20
#     else:
#         avg_frame = np.mean(frames, axis=0).astype(np.uint8)
#         hsv = cv2.cvtColor(avg_frame, cv2.COLOR_BGR2HSV)
#         roi_mask = get_workspace_roi_mask(avg_frame.shape)
        
#         s_channel = hsv[:, :, 1][roi_mask == 255]
#         v_channel = hsv[:, :, 2][roi_mask == 255]
        
#         valid_s = s_channel[s_channel > 15]
#         valid_v = v_channel[v_channel > 15]
        
#         min_s = int(np.percentile(valid_s, 10)) if len(valid_s) > 0 else 25
#         min_v = int(np.percentile(valid_v, 10)) if len(valid_v) > 0 else 20
        
#         min_s = max(20, min(min_s, 60))
#         min_v = max(20, min(min_v, 60))

#     return {
#         'red': [
#             ((0, min_s, min_v), (15, 255, 255)),
#             ((160, min_s, min_v), (180, 255, 255))
#         ],
#         'yellow': [((13, min_s, min_v), (35, 255, 255))],
#         'green':  [((36, min_s, min_v), (85, 255, 255))],
#         'blue':   [((86, min_s, min_v), (140, min_s, min_v))]
#     }

# def auto_find_empty_stack_zone(cap):
#     time.sleep(0.2)
#     ret, frame = cap.read()
#     if not ret or frame is None:
#         return 220.0, -100.0

#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     roi_mask = get_workspace_roi_mask(frame.shape)
#     combined_mask = np.zeros(hsv.shape[:2], dtype="uint8")

#     for color_name, ranges in COLOR_RANGES.items():
#         for lower, upper in ranges:
#             combined_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

#     kernel = np.ones((5, 5), np.uint8)
#     combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
#     combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

#     inv_mask = cv2.bitwise_not(combined_mask)
#     workspace_open = cv2.bitwise_and(inv_mask, inv_mask, mask=roi_mask)
#     dist_transform = cv2.distanceTransform(workspace_open, cv2.DIST_L2, 5)
    
#     _, max_val, _, max_loc = cv2.minMaxLoc(dist_transform)

#     if max_val > 25.0:
#         best_px, best_py = max_loc
#         rx, ry = pixels_to_robot_coords(best_px, best_py)
#         return rx, ry
#     return 220.0, -100.0

# def verify_stack_zone_has_block(cap):
#     time.sleep(0.3)
#     ret, frame = cap.read()
#     if not ret or frame is None:
#         return False

#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     stack_px, stack_py = robot_to_pixel_coords(STACK_X, STACK_Y)

#     mask_zone = np.zeros(hsv.shape[:2], dtype="uint8")
#     cv2.circle(mask_zone, (stack_px, stack_py), 40, 255, -1)

#     combined_color_mask = np.zeros(hsv.shape[:2], dtype="uint8")
#     for color_name, ranges in COLOR_RANGES.items():
#         for lower, upper in ranges:
#             combined_color_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

#     stack_colors = cv2.bitwise_and(combined_color_mask, combined_color_mask, mask=mask_zone)
#     return np.count_nonzero(stack_colors) > 200

# def show_preview(cap, current_tower_count=0, target_color=None, alert_msg=None):
#     ret, frame = cap.read()
#     if not ret or frame is None:
#         return None, None

#     preview_frame = frame.copy()
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     roi_mask = get_workspace_roi_mask(frame.shape)

#     cv2.polylines(preview_frame, [SRC_PIXELS.astype(np.int32)], True, (255, 100, 0), 1)
#     stack_px, stack_py = robot_to_pixel_coords(STACK_X, STACK_Y)
    
#     circle_color = (0, 255, 0) if current_tower_count >= TARGET_TOWER_HEIGHT else (255, 255, 255)
#     cv2.circle(preview_frame, (stack_px, stack_py), 45, circle_color, 2)

#     tower_banner = f"TOWER: {current_tower_count}/{TARGET_TOWER_HEIGHT} BLOCKS"
#     if target_color and current_tower_count < TARGET_TOWER_HEIGHT:
#         tower_banner += f" (NEXT: {target_color.upper()})"

#     cv2.rectangle(preview_frame, (10, 10), (450, 45), (40, 40, 40), -1)
#     banner_color = (0, 255, 0) if current_tower_count >= TARGET_TOWER_HEIGHT else (0, 215, 255)
#     cv2.putText(preview_frame, tower_banner, (20, 35),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.55, banner_color, 2)

#     ml_str = f"ML SAMPLES: {len(ml_corrector.samples_X)}"
#     cv2.putText(preview_frame, ml_str, (460, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

#     if alert_msg:
#         cv2.rectangle(preview_frame, (10, 50), (630, 90), (0, 0, 200), -1)
#         cv2.putText(preview_frame, alert_msg, (20, 78),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

#     found_target = None

#     for color_name, ranges in COLOR_RANGES.items():
#         mask = np.zeros(hsv.shape[:2], dtype="uint8")
#         for lower, upper in ranges:
#             mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

#         mask = cv2.bitwise_and(mask, roi_mask)
#         kernel = np.ones((5, 5), np.uint8)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for c in contours:
#             if cv2.contourArea(c) > 350:
#                 rect = cv2.minAreaRect(c)
#                 (cX, cY), (w, h), angle = rect

#                 if np.hypot(cX - stack_px, cY - stack_py) < 45:
#                     continue

#                 if w < h:
#                     angle = angle + 90.0

#                 box = np.int32(cv2.boxPoints(rect))
#                 bgr = COLOR_BGR.get(color_name, (0, 255, 0))
#                 cv2.drawContours(preview_frame, [box], 0, bgr, 2)
#                 cv2.circle(preview_frame, (int(cX), int(cY)), 4, bgr, -1)
                
#                 label = f"{color_name.upper()} {angle:.0f}deg"
#                 cv2.putText(preview_frame, label, (int(cX) - 30, int(cY) - 20),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 2)

#                 if target_color == color_name and found_target is None:
#                     found_target = (cX, cY, angle)

#     if found_target:
#         cv2.circle(preview_frame, (int(found_target[0]), int(found_target[1])), 12, (0, 255, 255), 3)

#     cv2.imshow("Dobot Vision Feed", preview_frame)
#     cv2.waitKey(1)
#     return frame, found_target

# def find_block_by_color(cap, color_name, current_tower_count=0):
#     target_data = None
#     for _ in range(6):
#         _, target_data = show_preview(cap, current_tower_count=current_tower_count, target_color=color_name)
#     return target_data

# def micro_align_above_target(cap, target_color, current_tower_count, nominal_rx, nominal_ry):
#     """
#     CLOSED-LOOP VISUAL MICRO-ALIGNMENT WITH ML FEEDBACK:
#     Measures visual error from hover position and feeds (nominal -> refined) pair to ML model.
#     """
#     time.sleep(0.2)
#     target_data = find_block_by_color(cap, target_color, current_tower_count=current_tower_count)
#     if target_data:
#         px, py, angle = target_data
#         refined_rx, refined_ry = pixels_to_robot_coords(px, py)
        
#         # Record observation to train ML Homing Correction model
#         ml_corrector.add_observation(nominal_rx, nominal_ry, refined_rx, refined_ry)
        
#         print(f"[CLOSED-LOOP ALIGN] Refined target to Robot:({refined_rx:.1f}, {refined_ry:.1f}) Angle:{angle:.1f}°")
#         return refined_rx, refined_ry, angle
#     return None, None, None

# def get_user_color_order():
#     valid_colors = ['red', 'green', 'blue', 'yellow']
#     print("\n" + "="*50)
#     print(" CHOOSE TOWER COLOR STACKING ORDER")
#     print(" Available options: red, green, blue, yellow")
#     print("="*50)
    
#     while True:
#         user_str = input("\nEnter 4 block colors in order (e.g., red, green, blue, yellow): ").strip().lower()
#         order = [c.strip() for c in user_str.split(',') if c.strip()]
        
#         if len(order) == 4 and all(c in valid_colors for c in order):
#             return order
        
#         print(f"[INVALID INPUT] Please enter exactly 4 valid colors separated by commas from: {', '.join(valid_colors)}")

# # ==========================================
# # 4. ROBOT HARDWARE & MONITORED MOTION
# # ==========================================

# def clear_robot_alarms(device):
#     print("[SYSTEM] Clearing active Dobot alarms...")
#     try:
#         if hasattr(device, 'clear_alarms'):
#             device.clear_alarms()
#         else:
#             msg = Message()
#             msg.id = 20
#             msg.ctrl = 0x01
#             msg.params = bytearray([0x00])
#             device._send_command(msg)
#         time.sleep(0.5)
#     except Exception as e:
#         print(f"[WARNING] Alarm clearance skipped: {e}")

# def get_dobot_port():
#     ports = list_ports.comports()
#     for p in ports:
#         dev = p.device
#         if "USB" in dev or "ACM" in dev or "ttyUSB" in dev or "ttyACM" in dev:
#             return dev
#     return None

# def home_robot(device):
#     """Triggers robot homing sequence and registers event with ML Homing Corrector."""
#     clear_robot_alarms(device)
#     print("[SYSTEM] Executing Dobot homing sequence... (~15s)")
    
#     # Register homing event in ML model
#     ml_corrector.register_homing_event()

#     try:
#         if hasattr(device, 'home'):
#             device.home()
#         else:
#             msg = Message()
#             msg.id = 31
#             msg.ctrl = 0x01
#             msg.params = bytearray([0x00, 0x00, 0x00, 0x00])
#             device._send_command(msg)
#             time.sleep(15)
            
#         device.speed(100, 100)
#         device.move_to(200.0, 0.0, SAFE_Z, 0.0, wait=False)
#         time.sleep(2.0)
#         print("[SYSTEM] Dobot homing complete!")
#     except Exception as e:
#         print(f"[WARNING] Homing sequence exception: {e}")

# def move_arm_monitored(device, cap, x, y, z, r=0.0, current_tower_count=0, target_color=None, target_duration=1.4):
#     device.move_to(x, y, z, r, wait=False)

#     start_time = time.time()
#     last_frame_gray = None
#     motion_history = []
#     stalled_counter = 0

#     while (time.time() - start_time) < target_duration:
#         frame, _ = show_preview(cap, current_tower_count=current_tower_count, target_color=target_color)
#         if frame is None:
#             time.sleep(0.05)
#             continue

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
#         if last_frame_gray is not None:
#             diff = cv2.absdiff(gray, last_frame_gray)
#             motion_pixels = np.sum(diff > 30)
#             motion_history.append(motion_pixels)

#             if len(motion_history) > 4 and max(motion_history[:3]) > 1500 and motion_pixels < 300:
#                 stalled_counter += 1
#             else:
#                 stalled_counter = 0

#             if stalled_counter >= 3:
#                 print("\n[ALARM DETECTED] Motor stall detected! Re-homing arm...")
#                 show_preview(cap, current_tower_count=current_tower_count, target_color=target_color, alert_msg="[ALARM] ARM STOPPED! CLEARING & RE-HOMING...")
                
#                 device.suck(False)
#                 clear_robot_alarms(device)
#                 home_robot(device)
#                 return False

#         last_frame_gray = gray
#         time.sleep(0.04)

#     return True

# # ==========================================
# # 5. MAIN EXECUTION WITH ML RECOVERY
# # ==========================================

# def main():
#     global COLOR_RANGES, STACK_X, STACK_Y, ADAPTIVE_PICK_Z_OFFSET, ADAPTIVE_SUCTION_DWELL, MISPLACEMENT_COUNT

#     print("\n" + "="*50)
#     print(" Dobot ML-Guided Homing Corrective Tower Stacker")
#     print("="*50)

#     cap, cam_idx = get_working_camera(override_index=CAMERA_INDEX_OVERRIDE)
#     if cap is None:
#         print("[ERROR] Could not connect to camera.")
#         return

#     cv2.namedWindow("Dobot Vision Feed", cv2.WINDOW_AUTOSIZE)

#     COLOR_RANGES = auto_calibrate_colors(cap)
#     STACK_X, STACK_Y = auto_find_empty_stack_zone(cap)
#     show_preview(cap, current_tower_count=0)

#     dobot_port = DOBOT_PORT_OVERRIDE if DOBOT_PORT_OVERRIDE is not None else get_dobot_port()
#     if not dobot_port:
#         print("[ERROR] No Dobot found on USB serial ports.")
#         cap.release()
#         cv2.destroyAllWindows()
#         return

#     print(f"[SYSTEM] Connecting to Dobot on port: {dobot_port}...")
#     try:
#         device = pydobot.Dobot(port=dobot_port, verbose=False)
#     except Exception as err:
#         print(f"[ERROR] Could not open serial port {dobot_port}: {err}")
#         cap.release()
#         cv2.destroyAllWindows()
#         return

#     # Perform Initial Homing (Registers Event #1 with ML Model)
#     home_robot(device)

#     stack_order = get_user_color_order()
#     print(f"\n[TARGET SEQUENCE] Selected Order: {' -> '.join([c.upper() for c in stack_order])}")

#     tower_count = 0

#     try:
#         while tower_count < TARGET_TOWER_HEIGHT:
#             target_color = stack_order[tower_count]
#             show_preview(cap, current_tower_count=tower_count, target_color=target_color)

#             target_data = find_block_by_color(cap, target_color, current_tower_count=tower_count)

#             if not target_data:
#                 print(f"\n[WAITING] Layer {tower_count + 1}/4: Looking for {target_color.upper()} block. Place it in view...")
#                 for _ in range(20):
#                     show_preview(cap, current_tower_count=tower_count, target_color=target_color)
#                     time.sleep(0.1)
#                 continue

#             px, py, block_angle = target_data
#             raw_rx, raw_ry = pixels_to_robot_coords(px, py)
            
#             # --- APPLY ML HOMING CORRECTION PREDICTION ---
#             ml_rx, ml_ry = ml_corrector.predict(raw_rx, raw_ry)

#             clear_robot_alarms(device)

#             # --- STEP 1: APPROACH WITH ML PREDICTED COORDINATES ---
#             if not move_arm_monitored(device, cap, ml_rx, ml_ry, SAFE_Z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
#                 continue

#             # --- STEP 2: HOVER ALIGNMENT & ML TRAINING FEEDBACK ---
#             refined_rx, refined_ry, refined_angle = micro_align_above_target(cap, target_color, tower_count, raw_rx, raw_ry)
#             if refined_rx is not None:
#                 ml_rx, ml_ry, block_angle = refined_rx, refined_ry, refined_angle

#             # --- STEP 3: PICKUP ---
#             effective_pick_z = BASE_PICK_Z + ADAPTIVE_PICK_Z_OFFSET
            
#             if not move_arm_monitored(device, cap, ml_rx, ml_ry, effective_pick_z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
#                 continue
            
#             device.suck(True)
#             time.sleep(ADAPTIVE_SUCTION_DWELL)

#             if not move_arm_monitored(device, cap, ml_rx, ml_ry, SAFE_Z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
#                 continue

#             # --- STEP 4: STACK PLACEMENT ---
#             current_stack_z = BASE_PICK_Z + (tower_count * BLOCK_HEIGHT)

#             if not move_arm_monitored(device, cap, STACK_X, STACK_Y, SAFE_Z, r=0.0, current_tower_count=tower_count, target_color=target_color):
#                 continue
#             if not move_arm_monitored(device, cap, STACK_X, STACK_Y, current_stack_z, r=0.0, current_tower_count=tower_count, target_color=target_color):
#                 continue

#             device.suck(False)
#             time.sleep(0.5)

#             if not move_arm_monitored(device, cap, STACK_X, STACK_Y, SAFE_Z, r=0.0, current_tower_count=tower_count, target_color=target_color):
#                 continue

#             # --- STEP 5: VERIFY & ADAPT ---
#             if verify_stack_zone_has_block(cap):
#                 tower_count += 1
#                 print(f"[SUCCESS] Stacked block #{tower_count}/{TARGET_TOWER_HEIGHT} ({target_color.upper()})!")
#             else:
#                 MISPLACEMENT_COUNT += 1
#                 print(f"\n[ADAPTATION] Misplacement detected on {target_color.upper()} block (#{MISPLACEMENT_COUNT})!")
                
#                 if ADAPTIVE_PICK_Z_OFFSET > -5.0:
#                     ADAPTIVE_PICK_Z_OFFSET -= 1.0
#                 ADAPTIVE_SUCTION_DWELL = min(1.2, ADAPTIVE_SUCTION_DWELL + 0.2)
                
#                 if MISPLACEMENT_COUNT % 2 == 0:
#                     COLOR_RANGES = auto_calibrate_colors(cap)

#         print("\n" + "="*50)
#         print(" [GOAL ACHIEVED] TOWER OF 4 BLOCKS SUCCESSFULLY BUILT!")
#         print(f" Sequence: {' -> '.join([c.upper() for c in stack_order])}")
#         print(f" ML Homing Correction Samples Collected: {len(ml_corrector.samples_X)}")
#         print("="*50)
        
#         move_arm_monitored(device, cap, 200.0, 0.0, SAFE_Z + 40.0, current_tower_count=tower_count)

#         while True:
#             show_preview(cap, current_tower_count=4)
#             time.sleep(0.05)

#     finally:
#         cv2.destroyAllWindows()
#         cap.release()
#         try:
#             device.suck(False)
#             device.close()
#         except NameError:
#             pass
#         print("[SYSTEM] Resources released.")

# if __name__ == "__main__":
#     main()


import os
# Suppress OpenCV & Qt C++ level warning logs in terminal
os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["QT_LOGGING_RULES"] = "*=false"

import cv2
import glob
import time
import numpy as np
import pydobot
from pydobot.message import Message
from serial.tools import list_ports

# ==========================================
# 1. HARDWARE & ADAPTIVE CONFIGURATION
# ==========================================

CAMERA_INDEX_OVERRIDE = None
DOBOT_PORT_OVERRIDE = None
TARGET_TOWER_HEIGHT = 4

# Physical Dobot Arm Heights (in mm)
SAFE_Z = 25.0        # Safe clearance height above blocks
BASE_PICK_Z = -45.0  # Default pickup Z height
BLOCK_HEIGHT = 25.0  # Height of a single block for stacking offset

# Adaptive Offset Parameters
ADAPTIVE_PICK_Z_OFFSET = 0.0  
ADAPTIVE_SUCTION_DWELL = 0.5   
MISPLACEMENT_COUNT = 0         

STACK_X = 220.0
STACK_Y = -100.0

# ------------------------------------------------------------------
# Perspective Calibration Points (4 Workspace Corners)
# ------------------------------------------------------------------
SRC_PIXELS = np.array([
    [100, 100],  # Top-Left Corner
    [540, 100],  # Top-Right Corner
    [540, 380],  # Bottom-Right Corner
    [100, 380]   # Bottom-Left Corner
], dtype=np.float32)

DST_ROBOT = np.array([
    [150.0,  120.0],  # Top-Left Robot (X, Y)
    [150.0, -120.0],  # Top-Right Robot (X, Y)
    [320.0, -120.0],  # Bottom-Right Robot (X, Y)
    [320.0,  120.0]   # Bottom-Left Robot (X, Y)
], dtype=np.float32)

H_MATRIX, _ = cv2.findHomography(SRC_PIXELS, DST_ROBOT)
H_INVERSE, _ = cv2.findHomography(DST_ROBOT, SRC_PIXELS)

COLOR_RANGES = {}
COLOR_BGR = {
    'red':    (0, 0, 255),
    'green':  (0, 255, 0),
    'blue':   (255, 0, 0),
    'yellow': (0, 255, 255)
}

# ==========================================
# 2. AGGRESSIVE MACHINE LEARNING MODEL
# ==========================================

class HomingCorrectionML:
    """
    High-Gain / High-Loss Aggressive Online Regression Model.
    Learns mechanical homing hysteresis and applies 1.8x aggressive step corrections.
    """
    def __init__(self):
        self.samples_X = []  
        self.samples_Y = []  
        self.homing_count = 0
        self.is_trained = False
        
        self.AGGRESSION_GAIN = 1.8   
        self.RECENCY_GAMMA = 2.5     
        self.latest_loss = 0.0       
        
        try:
            from sklearn.linear_model import Ridge
            self.model = Ridge(alpha=1e-4)
            self.use_sklearn = True
            print("[ML SYSTEM] Aggressive Scikit-Learn Ridge model initialized.")
        except ImportError:
            self.use_sklearn = False
            print("[ML SYSTEM] Scikit-Learn not found. Using NumPy Least-Squares.")

    def register_homing_event(self):
        self.homing_count += 1
        print(f"\n[ML SYSTEM] Homing Event #{self.homing_count} recorded.")

    def add_observation(self, nominal_x, nominal_y, actual_x, actual_y):
        dx = actual_x - nominal_x
        dy = actual_y - nominal_y
        
        self.samples_X.append([nominal_x, nominal_y, float(self.homing_count)])
        self.samples_Y.append([dx, dy])
        self.latest_loss = float(np.mean(np.square([dx, dy])))
        
        if len(self.samples_X) >= 1:
            self.train()

    def train(self):
        X = np.array(self.samples_X)
        Y = np.array(self.samples_Y)
        N = len(X)
        weights = np.array([self.RECENCY_GAMMA ** (i - N + 1) for i in range(N)])
        
        if self.use_sklearn:
            self.model.fit(X, Y, sample_weight=weights)
        self.is_trained = True

    def predict(self, nominal_x, nominal_y):
        if not self.is_trained or len(self.samples_X) < 1:
            return nominal_x, nominal_y
        
        feat = np.array([[nominal_x, nominal_y, float(self.homing_count)]])
        
        if self.use_sklearn:
            pred_offset = self.model.predict(feat)[0]
        else:
            X = np.array(self.samples_X)
            Y = np.array(self.samples_Y)
            W, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
            pred_offset = (feat @ W)[0]

        corr_x = nominal_x + (pred_offset[0] * self.AGGRESSION_GAIN)
        corr_y = nominal_y + (pred_offset[1] * self.AGGRESSION_GAIN)
        return corr_x, corr_y

ml_corrector = HomingCorrectionML()

# ==========================================
# 3. HOMOGRAPHY & VISION FUNCTIONS
# ==========================================

def pixels_to_robot_coords(px, py):
    pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(pt, H_MATRIX)
    return float(transformed[0][0][0]), float(transformed[0][0][1])

def robot_to_pixel_coords(rx, ry):
    pt = np.array([[[float(rx), float(ry)]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(pt, H_INVERSE)
    return int(round(transformed[0][0][0])), int(round(transformed[0][0][1]))

def list_system_cameras():
    print("[SYSTEM] Detecting V4L2 camera devices:")
    devices = sorted(glob.glob('/sys/class/video4linux/video*'))
    for dev in devices:
        try:
            with open(f"{dev}/name", "r") as f:
                name = f.read().strip()
            idx = int(dev.replace('/sys/class/video4linux/video', ''))
            print(f"  -> Index {idx}: {name}")
        except Exception:
            pass

def get_working_camera(override_index=None):
    list_system_cameras()

    def test_camera_index(idx):
        print(f"[SYSTEM] Testing camera index {idx}...")
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        valid_frame = False
        for _ in range(25):
            ret, frame = cap.read()
            if ret and frame is not None and frame.mean() > 10.0:
                valid_frame = True
                break
            time.sleep(0.04)

        if valid_frame:
            return cap

        cap.release()
        return None

    if override_index is not None:
        cap = test_camera_index(override_index)
        if cap:
            return cap, override_index

    for idx in [2, 4, 6, 0]:
        cap = test_camera_index(idx)
        if cap:
            return cap, idx

    return None, None

def get_workspace_roi_mask(shape):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    pts = SRC_PIXELS.astype(np.int32)
    cv2.fillPoly(mask, [pts], 255)
    return mask

def auto_calibrate_colors(cap):
    print("[SYSTEM] Auto-calibrating multi-tone color thresholds...")
    frames = []
    for _ in range(10):
        ret, frame = cap.read()
        if ret and frame is not None:
            frames.append(frame)
        time.sleep(0.04)
        
    if not frames:
        min_s, min_v = 25, 20
    else:
        avg_frame = np.mean(frames, axis=0).astype(np.uint8)
        hsv = cv2.cvtColor(avg_frame, cv2.COLOR_BGR2HSV)
        roi_mask = get_workspace_roi_mask(avg_frame.shape)
        
        s_channel = hsv[:, :, 1][roi_mask == 255]
        v_channel = hsv[:, :, 2][roi_mask == 255]
        
        valid_s = s_channel[s_channel > 15]
        valid_v = v_channel[v_channel > 15]
        
        min_s = int(np.percentile(valid_s, 10)) if len(valid_s) > 0 else 25
        min_v = int(np.percentile(valid_v, 10)) if len(valid_v) > 0 else 20
        
        min_s = max(20, min(min_s, 60))
        min_v = max(20, min(min_v, 60))

    return {
        'red': [
            ((0, min_s, min_v), (15, 255, 255)),
            ((160, min_s, min_v), (180, 255, 255))
        ],
        'yellow': [((13, min_s, min_v), (35, 255, 255))],
        'green':  [((36, min_s, min_v), (85, 255, 255))],
        'blue':   [((86, min_s, min_v), (140, 255, 255))]
    }

def auto_find_empty_stack_zone(cap):
    time.sleep(0.2)
    ret, frame = cap.read()
    if not ret or frame is None:
        return 220.0, -100.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi_mask = get_workspace_roi_mask(frame.shape)
    combined_mask = np.zeros(hsv.shape[:2], dtype="uint8")

    for color_name, ranges in COLOR_RANGES.items():
        for lower, upper in ranges:
            combined_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

    inv_mask = cv2.bitwise_not(combined_mask)
    workspace_open = cv2.bitwise_and(inv_mask, inv_mask, mask=roi_mask)
    dist_transform = cv2.distanceTransform(workspace_open, cv2.DIST_L2, 5)
    
    _, max_val, _, max_loc = cv2.minMaxLoc(dist_transform)

    if max_val > 25.0:
        best_px, best_py = max_loc
        rx, ry = pixels_to_robot_coords(best_px, best_py)
        return rx, ry
    return 220.0, -100.0

def verify_pickup_failed(cap, target_color, pick_px, pick_py, radius=35):
    """
    VISUAL PICKUP VERIFICATION:
    Inspects target pixel location after suction attempt.
    Returns True if target_color is STILL present on workspace (Pickup Failed).
    """
    time.sleep(0.25)
    
    # Flush video buffer to grab fresh frame
    for _ in range(4):
        ret, frame = cap.read()
        
    if not ret or frame is None:
        return False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi_mask = get_workspace_roi_mask(frame.shape)

    # ROI circle around pickup point
    pickup_mask = np.zeros(hsv.shape[:2], dtype="uint8")
    cv2.circle(pickup_mask, (int(pick_px), int(pick_py)), radius, 255, -1)

    # Color threshold for target color
    target_color_mask = np.zeros(hsv.shape[:2], dtype="uint8")
    if target_color in COLOR_RANGES:
        for lower, upper in COLOR_RANGES[target_color]:
            target_color_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

    target_color_mask = cv2.bitwise_and(target_color_mask, roi_mask)
    remaining_color_pixels = cv2.bitwise_and(target_color_mask, target_color_mask, mask=pickup_mask)
    
    pixel_count = np.count_nonzero(remaining_color_pixels)
    return pixel_count > 150

def verify_stack_zone_has_block(cap):
    time.sleep(0.3)
    ret, frame = cap.read()
    if not ret or frame is None:
        return False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    stack_px, stack_py = robot_to_pixel_coords(STACK_X, STACK_Y)

    mask_zone = np.zeros(hsv.shape[:2], dtype="uint8")
    cv2.circle(mask_zone, (stack_px, stack_py), 40, 255, -1)

    combined_color_mask = np.zeros(hsv.shape[:2], dtype="uint8")
    for color_name, ranges in COLOR_RANGES.items():
        for lower, upper in ranges:
            combined_color_mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

    stack_colors = cv2.bitwise_and(combined_color_mask, combined_color_mask, mask=mask_zone)
    return np.count_nonzero(stack_colors) > 200

def show_preview(cap, current_tower_count=0, target_color=None, alert_msg=None):
    ret, frame = cap.read()
    if not ret or frame is None:
        return None, None

    preview_frame = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi_mask = get_workspace_roi_mask(frame.shape)

    cv2.polylines(preview_frame, [SRC_PIXELS.astype(np.int32)], True, (255, 100, 0), 1)
    stack_px, stack_py = robot_to_pixel_coords(STACK_X, STACK_Y)
    
    circle_color = (0, 255, 0) if current_tower_count >= TARGET_TOWER_HEIGHT else (255, 255, 255)
    cv2.circle(preview_frame, (stack_px, stack_py), 45, circle_color, 2)

    tower_banner = f"TOWER: {current_tower_count}/{TARGET_TOWER_HEIGHT} BLOCKS"
    if target_color and current_tower_count < TARGET_TOWER_HEIGHT:
        tower_banner += f" (NEXT: {target_color.upper()})"

    cv2.rectangle(preview_frame, (10, 10), (450, 45), (40, 40, 40), -1)
    banner_color = (0, 255, 0) if current_tower_count >= TARGET_TOWER_HEIGHT else (0, 215, 255)
    cv2.putText(preview_frame, tower_banner, (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, banner_color, 2)

    ml_str = f"AGGRESSIVE ML | N={len(ml_corrector.samples_X)} | LOSS:{ml_corrector.latest_loss:.2f}"
    cv2.putText(preview_frame, ml_str, (320, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)

    if alert_msg:
        cv2.rectangle(preview_frame, (10, 50), (630, 90), (0, 0, 200), -1)
        cv2.putText(preview_frame, alert_msg, (20, 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    found_target = None

    for color_name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype="uint8")
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))

        mask = cv2.bitwise_and(mask, roi_mask)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            if cv2.contourArea(c) > 350:
                rect = cv2.minAreaRect(c)
                (cX, cY), (w, h), angle = rect

                if np.hypot(cX - stack_px, cY - stack_py) < 45:
                    continue

                if w < h:
                    angle = angle + 90.0

                box = np.int32(cv2.boxPoints(rect))
                bgr = COLOR_BGR.get(color_name, (0, 255, 0))
                cv2.drawContours(preview_frame, [box], 0, bgr, 2)
                cv2.circle(preview_frame, (int(cX), int(cY)), 4, bgr, -1)
                
                label = f"{color_name.upper()} {angle:.0f}deg"
                cv2.putText(preview_frame, label, (int(cX) - 30, int(cY) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 2)

                if target_color == color_name and found_target is None:
                    found_target = (cX, cY, angle)

    if found_target:
        cv2.circle(preview_frame, (int(found_target[0]), int(found_target[1])), 12, (0, 255, 255), 3)

    cv2.imshow("Dobot Vision Feed", preview_frame)
    cv2.waitKey(1)
    return frame, found_target

def find_block_by_color(cap, color_name, current_tower_count=0):
    target_data = None
    for _ in range(6):
        _, target_data = show_preview(cap, current_tower_count=current_tower_count, target_color=color_name)
    return target_data

def micro_align_above_target(cap, target_color, current_tower_count, nominal_rx, nominal_ry):
    time.sleep(0.2)
    target_data = find_block_by_color(cap, target_color, current_tower_count=current_tower_count)
    if target_data:
        px, py, angle = target_data
        refined_rx, refined_ry = pixels_to_robot_coords(px, py)
        ml_corrector.add_observation(nominal_rx, nominal_ry, refined_rx, refined_ry)
        return refined_rx, refined_ry, angle
    return None, None, None

def get_user_color_order():
    valid_colors = ['red', 'green', 'blue', 'yellow']
    print("\n" + "="*50)
    print(" CHOOSE TOWER COLOR STACKING ORDER")
    print(" Available options: red, green, blue, yellow")
    print("="*50)
    
    while True:
        user_str = input("\nEnter 4 block colors in order (e.g., red, green, blue, yellow): ").strip().lower()
        order = [c.strip() for c in user_str.split(',') if c.strip()]
        
        if len(order) == 4 and all(c in valid_colors for c in order):
            return order
        
        print(f"[INVALID INPUT] Please enter 4 valid colors from: {', '.join(valid_colors)}")

# ==========================================
# 4. ROBOT HARDWARE & MONITORED MOTION
# ==========================================

def clear_robot_alarms(device):
    try:
        if hasattr(device, 'clear_alarms'):
            device.clear_alarms()
        else:
            msg = Message()
            msg.id = 20
            msg.ctrl = 0x01
            msg.params = bytearray([0x00])
            device._send_command(msg)
        time.sleep(0.5)
    except Exception as e:
        print(f"[WARNING] Alarm clearance skipped: {e}")

def get_dobot_port():
    ports = list_ports.comports()
    for p in ports:
        dev = p.device
        if "USB" in dev or "ACM" in dev or "ttyUSB" in dev or "ttyACM" in dev:
            return dev
    return None

def home_robot(device):
    clear_robot_alarms(device)
    print("[SYSTEM] Executing Dobot homing sequence... (~15s)")
    ml_corrector.register_homing_event()

    try:
        if hasattr(device, 'home'):
            device.home()
        else:
            msg = Message()
            msg.id = 31
            msg.ctrl = 0x01
            msg.params = bytearray([0x00, 0x00, 0x00, 0x00])
            device._send_command(msg)
            time.sleep(15)
            
        device.speed(100, 100)
        device.move_to(200.0, 0.0, SAFE_Z, 0.0, wait=False)
        time.sleep(2.0)
        print("[SYSTEM] Dobot homing complete!")
    except Exception as e:
        print(f"[WARNING] Homing sequence exception: {e}")

def move_arm_monitored(device, cap, x, y, z, r=0.0, current_tower_count=0, target_color=None, target_duration=1.4):
    device.move_to(x, y, z, r, wait=False)

    start_time = time.time()
    last_frame_gray = None
    motion_history = []
    stalled_counter = 0

    while (time.time() - start_time) < target_duration:
        frame, _ = show_preview(cap, current_tower_count=current_tower_count, target_color=target_color)
        if frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if last_frame_gray is not None:
            diff = cv2.absdiff(gray, last_frame_gray)
            motion_pixels = np.sum(diff > 30)
            motion_history.append(motion_pixels)

            if len(motion_history) > 4 and max(motion_history[:3]) > 1500 and motion_pixels < 300:
                stalled_counter += 1
            else:
                stalled_counter = 0

            if stalled_counter >= 3:
                print("\n[ALARM DETECTED] Motor stall detected! Re-homing arm...")
                show_preview(cap, current_tower_count=current_tower_count, target_color=target_color, alert_msg="[ALARM] ARM STOPPED! CLEARING & RE-HOMING...")
                
                device.suck(False)
                clear_robot_alarms(device)
                home_robot(device)
                return False

        last_frame_gray = gray
        time.sleep(0.04)

    return True

# ==========================================
# 5. MAIN EXECUTION WITH VISUAL PICKUP CHECK
# ==========================================

def main():
    global COLOR_RANGES, STACK_X, STACK_Y, ADAPTIVE_PICK_Z_OFFSET, ADAPTIVE_SUCTION_DWELL, MISPLACEMENT_COUNT

    print("\n" + "="*50)
    print(" Dobot ML Stacker with Visual Pickup Verification")
    print("="*50)

    cap, cam_idx = get_working_camera(override_index=CAMERA_INDEX_OVERRIDE)
    if cap is None:
        print("[ERROR] Could not connect to camera.")
        return

    cv2.namedWindow("Dobot Vision Feed", cv2.WINDOW_AUTOSIZE)

    COLOR_RANGES = auto_calibrate_colors(cap)
    STACK_X, STACK_Y = auto_find_empty_stack_zone(cap)
    show_preview(cap, current_tower_count=0)

    dobot_port = DOBOT_PORT_OVERRIDE if DOBOT_PORT_OVERRIDE is not None else get_dobot_port()
    if not dobot_port:
        print("[ERROR] No Dobot found on USB serial ports.")
        cap.release()
        cv2.destroyAllWindows()
        return

    print(f"[SYSTEM] Connecting to Dobot on port: {dobot_port}...")
    try:
        device = pydobot.Dobot(port=dobot_port, verbose=False)
    except Exception as err:
        print(f"[ERROR] Could not open serial port {dobot_port}: {err}")
        cap.release()
        cv2.destroyAllWindows()
        return

    home_robot(device)

    stack_order = get_user_color_order()
    print(f"\n[TARGET SEQUENCE] Selected Order: {' -> '.join([c.upper() for c in stack_order])}")

    tower_count = 0

    try:
        while tower_count < TARGET_TOWER_HEIGHT:
            target_color = stack_order[tower_count]
            show_preview(cap, current_tower_count=tower_count, target_color=target_color)

            target_data = find_block_by_color(cap, target_color, current_tower_count=tower_count)

            if not target_data:
                print(f"\n[WAITING] Layer {tower_count + 1}/4: Looking for {target_color.upper()} block. Place it in view...")
                for _ in range(20):
                    show_preview(cap, current_tower_count=tower_count, target_color=target_color)
                    time.sleep(0.1)
                continue

            px, py, block_angle = target_data
            raw_rx, raw_ry = pixels_to_robot_coords(px, py)
            
            ml_rx, ml_ry = ml_corrector.predict(raw_rx, raw_ry)

            clear_robot_alarms(device)

            # --- STEP 1: APPROACH ---
            if not move_arm_monitored(device, cap, ml_rx, ml_ry, SAFE_Z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
                continue

            # --- STEP 2: HOVER ALIGNMENT ---
            refined_rx, refined_ry, refined_angle = micro_align_above_target(cap, target_color, tower_count, raw_rx, raw_ry)
            if refined_rx is not None:
                ml_rx, ml_ry, block_angle = refined_rx, refined_ry, refined_angle

            # --- STEP 3: PICKUP ATTEMPT ---
            effective_pick_z = BASE_PICK_Z + ADAPTIVE_PICK_Z_OFFSET
            
            if not move_arm_monitored(device, cap, ml_rx, ml_ry, effective_pick_z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
                continue
            
            device.suck(True)
            time.sleep(ADAPTIVE_SUCTION_DWELL)

            if not move_arm_monitored(device, cap, ml_rx, ml_ry, SAFE_Z, r=block_angle, current_tower_count=tower_count, target_color=target_color):
                continue

            # --- STEP 3.5: VISUAL PICKUP VERIFICATION ---
            pick_px, pick_py = robot_to_pixel_coords(ml_rx, ml_ry)
            if verify_pickup_failed(cap, target_color, pick_px, pick_py):
                print(f"\n[PICKUP FAILED] Block {target_color.upper()} is STILL detected at ({ml_rx:.1f}, {ml_ry:.1f})!")
                show_preview(cap, current_tower_count=tower_count, target_color=target_color, alert_msg=f"[FAILED PICKUP] {target_color.upper()} STILL IN PLACE! RETRYING...")
                
                device.suck(False)
                MISPLACEMENT_COUNT += 1
                
                # Dynamically deepen reach and extend suction dwell
                if ADAPTIVE_PICK_Z_OFFSET > -5.0:
                    ADAPTIVE_PICK_Z_OFFSET -= 1.0
                ADAPTIVE_SUCTION_DWELL = min(1.2, ADAPTIVE_SUCTION_DWELL + 0.2)
                
                print(f" -> Adjusted Z Offset: {ADAPTIVE_PICK_Z_OFFSET:.1f}mm | Dwell: {ADAPTIVE_SUCTION_DWELL:.1f}s. Retrying pickup...")
                time.sleep(1.0)
                continue  # Retry pickup immediately without moving to stack area

            # --- STEP 4: STACK PLACEMENT ---
            current_stack_z = BASE_PICK_Z + (tower_count * BLOCK_HEIGHT)

            if not move_arm_monitored(device, cap, STACK_X, STACK_Y, SAFE_Z, r=0.0, current_tower_count=tower_count, target_color=target_color):
                continue
            if not move_arm_monitored(device, cap, STACK_X, STACK_Y, current_stack_z, r=0.0, current_tower_count=tower_count, target_color=target_color):
                continue

            device.suck(False)
            time.sleep(0.5)

            if not move_arm_monitored(device, cap, STACK_X, STACK_Y, SAFE_Z, r=0.0, current_tower_count=tower_count, target_color=target_color):
                continue

            # --- STEP 5: VERIFY STACK PLACEMENT ---
            if verify_stack_zone_has_block(cap):
                tower_count += 1
                print(f"[SUCCESS] Stacked block #{tower_count}/{TARGET_TOWER_HEIGHT} ({target_color.upper()})!")
            else:
                MISPLACEMENT_COUNT += 1
                print(f"\n[ADAPTATION] Stack placement failed for {target_color.upper()} block (#{MISPLACEMENT_COUNT})!")
                
                if ADAPTIVE_PICK_Z_OFFSET > -5.0:
                    ADAPTIVE_PICK_Z_OFFSET -= 1.0
                ADAPTIVE_SUCTION_DWELL = min(1.2, ADAPTIVE_SUCTION_DWELL + 0.2)

        print("\n" + "="*50)
        print(" [GOAL ACHIEVED] TOWER OF 4 BLOCKS SUCCESSFULLY BUILT!")
        print(f" Sequence: {' -> '.join([c.upper() for c in stack_order])}")
        print("="*50)
        
        move_arm_monitored(device, cap, 200.0, 0.0, SAFE_Z + 40.0, current_tower_count=tower_count)

        while True:
            show_preview(cap, current_tower_count=4)
            time.sleep(0.05)

    finally:
        cv2.destroyAllWindows()
        cap.release()
        try:
            device.suck(False)
            device.close()
        except NameError:
            pass
        print("[SYSTEM] Hardware resources released.")

if __name__ == "__main__":
    main()