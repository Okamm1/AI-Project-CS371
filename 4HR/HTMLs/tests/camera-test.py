import cv2
import time
from picarx import Picarx

px = Picarx()

# Servo scan ranges
pan_range = range(-40, 41, 5)    # left → right
tilt_range = range(30, -31, -5)  # up → down

cap = cv2.VideoCapture(0)

def find_red_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = (0, 100, 100)
    upper_red = (10, 255, 255)
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    moments = cv2.moments(mask)
    
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return (cx, cy)
    
    return None

frame_center = (320, 240)  # adjust if needed

best_pan = 0
best_tilt = 0
min_error = float('inf')

print("Starting camera calibration scan...")

for pan in pan_range:
    px.set_cam_pan_angle(pan)
    time.sleep(0.3)

    for tilt in tilt_range:
        px.set_cam_tilt_angle(tilt)
        time.sleep(0.3)

        ret, frame = cap.read()
        if not ret:
            continue

        center = find_red_center(frame)

        if center:
            error = abs(center[0] - frame_center[0]) + abs(center[1] - frame_center[1])

            print(f"Pan {pan}, Tilt {tilt}, Error {error}")

            if error < min_error:
                min_error = error
                best_pan = pan
                best_tilt = tilt

cap.release()

print("\n=== BEST CALIBRATION ===")
print(f"Best pan (forward): {best_pan}")
print(f"Best tilt (horizon): {best_tilt}")

px.set_cam_pan_angle(best_pan)
px.set_cam_tilt_angle(best_tilt)

# Save results
with open("camera_calibration.txt", "w") as f:
    f.write(f"{best_pan},{best_tilt}")

print("Saved to camera_calibration.txt")