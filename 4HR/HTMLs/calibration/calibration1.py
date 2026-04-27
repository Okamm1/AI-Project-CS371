#!/usr/bin/env python3
from picarx import Picarx
from time import sleep
import readchar

px = Picarx()

print("""
========= PiCar-X CALIBRATION TOOL =========

1 -> Steering (drive straight)
2 -> Camera Pan (look forward)
3 -> Camera Tilt (level horizon)

W/D -> increase angle
S/A -> decrease angle

R -> test movement
SPACE -> save calibration
CTRL+C -> exit

===========================================
""")

servo = 0  # 0=steering, 1=pan, 2=tilt
offsets = [0, 0, 0]

step = 1


# -----------------------------
# APPLY SERVO OFFSET
# -----------------------------
def apply_offset():
    px.dir_cali_val = offsets[0]
    px.cam_pan_cali_val = offsets[1]
    px.cam_tilt_cali_val = offsets[2]

    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)


# -----------------------------
# TEST MOVEMENT (REQUIRED)
# -----------------------------
def test_movement():
    print("Testing movement...")

    # Forward straight
    px.set_dir_servo_angle(0)
    px.forward(30)
    sleep(1)

    # Forward right
    px.set_dir_servo_angle(10)
    px.forward(30)
    sleep(1)

    # Forward left
    px.set_dir_servo_angle(-10)
    px.forward(30)
    sleep(1)

    px.stop()
    sleep(0.5)

    # Backward straight
    px.set_dir_servo_angle(0)
    px.backward(30)
    sleep(1)

    # Backward right
    px.set_dir_servo_angle(10)
    px.backward(30)
    sleep(1)

    # Backward left
    px.set_dir_servo_angle(-10)
    px.backward(30)
    sleep(1)

    px.stop()


# -----------------------------
# DISPLAY INFO
# -----------------------------
def show():
    names = ["Steering", "Camera Pan", "Camera Tilt"]
    print("\033[H\033[J", end="")
    print(f"Selected: {names[servo]}")
    print(f"Offsets: {offsets}")


# -----------------------------
# MAIN LOOP
# -----------------------------
def calibrate():
    global servo

    show()

    while True:
        key = readchar.readkey().lower()

        if key in ('1', '2', '3'):
            servo = int(key) - 1
            show()

        elif key in ('w', 'd'):
            offsets[servo] += step
            apply_offset()
            show()

        elif key in ('s', 'a'):
            offsets[servo] -= step
            apply_offset()
            show()

        elif key == 'r':
            test_movement()
            show()

        elif key == readchar.key.SPACE:
            print("Saving calibration...")

            px.dir_servo_calibrate(offsets[0])
            px.cam_pan_servo_calibrate(offsets[1])
            px.cam_tilt_servo_calibrate(offsets[2])

            print("Calibration saved.")
            sleep(1)
            show()

        elif key == readchar.key.CTRL_C:
            print("Exiting...")
            break


if __name__ == "__main__":
    try:
        calibrate()
    finally:
        px.stop()
Calibration file
