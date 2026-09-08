import cv2
import numpy as np
import pydobot
from serial.tools import list_ports
import time

# ==========================================
# 1. CONFIGURATION & CALIBRATION
# ==========================================

# Define HSV color boundaries for the blocks (Adjust these based on your room lighting)
COLOR_RANGES = {
    'red':   [(0, 120, 70), (10, 255, 255)], # Red can also wrap around to 170-180 in OpenCV
    'green': [(40, 50, 50), (80, 255, 255)],
    'blue':  [(100, 150, 0), (140, 255, 255)],
    'yellow':[(20, 100, 100), (30, 255, 255)]
}

# Stacking Base Coordinates (Where the tower will be built)
STACK_X = 200.0
STACK_Y = 0.0
BASE_Z = -40.0       # Z-height of the table/surface
BLOCK_HEIGHT = 25.0  # Height of a single block in mm
SAFE_Z = 50.0        # Safe height to travel without hitting anything

# ==========================================
# 2. HELPER FUNCTIONS
# ==========================================

def pixels_to_robot_coords(px, py):
    """
    Translates camera pixel coordinates (x, y) to Dobot physical coordinates (X, Y).
    NOTE: This is a simplified linear mapping. In reality, you will need to map 
    4 corners of your camera view to 4 known Dobot coordinates using cv2.getPerspectiveTransform.
    """
    # PLACEHOLDER CALIBRATION: You must calculate your scale and offsets!
    scale_x = 0.5  # mm per pixel
    scale_y = 0.5  # mm per pixel
    offset_x = 100 # Robot X offset from camera origin
    offset_y = -50 # Robot Y offset from camera origin
    
    robot_x = (py * scale_x) + offset_x # Note: Camera Y is often Robot X depending on setup
    robot_y = (px * scale_y) + offset_y
    
    return robot_x, robot_y

def find_block_by_color(frame, color_name):
    """Finds the center pixel (X, Y) of the largest block of the requested color."""
    if color_name not in COLOR_RANGES:
        print(f"Color {color_name} not defined.")
        return None
        
    lower, upper = COLOR_RANGES[color_name]
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")
    
    # Convert to HSV and apply mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    
    # Clean up the mask
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        # Find the largest contour (the block)
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        
        # Calculate center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return cX, cY
            
    return None

# ==========================================
# 3. MAIN LOGIC
# ==========================================

def main():
    # 1. Connect to Dobot
    available_ports = list_ports.comports()
    if not available_ports:
        print("No Dobot found. Check USB connection.")
        return
        
    dobot_port = available_ports[0].device
    print(f"Connecting to Dobot on {dobot_port}...")
    device = pydobot.Dobot(port=dobot_port, verbose=False)
    
    # 2. Initialize Camera
    cap = cv2.VideoCapture(0) # Use 1 or 2 if using an external USB webcam
    time.sleep(2) # Camera warmup
    
    # 3. Get User Input
    print("\nAvailable colors: red, green, blue, yellow")
    user_input = input("Enter the order to stack (e.g., red, blue, green): ")
    stack_order = [color.strip().lower() for color in user_input.split(',')]
    
    # 4. Execute the Pick and Place sequence
    for index, color in enumerate(stack_order):
        print(f"\nLooking for {color} block...")
        
        # Grab a few frames to clear buffer and get a fresh image
        for _ in range(5):
            ret, frame = cap.read()
            
        block_pixels = find_block_by_color(frame, color)
        
        if not block_pixels:
            print(f"Could not find {color} block! Skipping...")
            continue
            
        px, py = block_pixels
        robot_x, robot_y = pixels_to_robot_coords(px, py)
        print(f"Found {color} at pixels: ({px}, {py}) -> Robot Coords: ({robot_x:.1f}, {robot_y:.1f})")
        
        # Calculate stacking height (Z increases with each block)
        current_place_z = BASE_Z + (index * BLOCK_HEIGHT)
        
        # --- KINEMATICS SEQUENCE ---
        # Move above the block
        device.move_to(robot_x, robot_y, SAFE_Z, 0, wait=True)
        # Drop down to block
        device.move_to(robot_x, robot_y, BASE_Z, 0, wait=True)
        # Turn on suction
        device.suck(True)
        time.sleep(0.5)
        # Pull straight up
        device.move_to(robot_x, robot_y, SAFE_Z, 0, wait=True)
        
        # Move above stack
        device.move_to(STACK_X, STACK_Y, SAFE_Z, 0, wait=True)
        # Drop down to stack height
        device.move_to(STACK_X, STACK_Y, current_place_z, 0, wait=True)
        # Turn off suction
        device.suck(False)
        time.sleep(0.5)
        # Pull straight up
        device.move_to(STACK_X, STACK_Y, SAFE_Z, 0, wait=True)
        
        print(f"Successfully stacked {color}.")

    # Cleanup
    cap.release()
    device.close()
    print("\nTask Complete!")

if __name__ == "__main__":
    main()