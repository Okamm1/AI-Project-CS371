#!/usr/bin/env python3
"""
PiCar-X Full Pipeline
=====================
Phase 1: Calibration  — tune servos, save offsets
Phase 2: Line Follow  — PID steering, dynamic speed,
                        90-degree turns, intersection handling,
                        and line-loss recovery
"""

from picarx import Picarx
from time import sleep, time
import readchar

px = Picarx()

# ─────────────────────────────────────────────
# TUNABLE CONSTANTS  (adjust for your track)
# ─────────────────────────────────────────────

# ── Grayscale sensor thresholds ──
DARK_THRESHOLD   = 300   # below = on the line (dark)
SENSOR_WEIGHTS   = (-1.0, 0.0, 1.0)  # left, center, right

# ── PID gains ──
KP = 28.0   # proportional  (main steering force)
KI =  0.3   # integral      (corrects persistent drift)
KD =  8.0   # derivative    (dampens oscillation)

MAX_STEERING = 35   # degrees, hard cap on servo angle

# ── Speed ──
BASE_SPEED      = 25   # normal cruising speed (0–100)
MIN_SPEED       = 12   # minimum speed while turning hard
SPEED_REDUCTION =  0.5 # how aggressively speed drops with error
                        # drive_speed = BASE * (1 - SPEED_REDUCTION * |error|)

# ── Intersection / turn detection ──
INTERSECTION_TIME  = 0.4   # seconds all 3 sensors must be dark to confirm intersection
TURN_DRIVE_TIME    = 0.45  # seconds to drive forward before executing a 90° turn
TURN_EXECUTE_TIME  = 0.9   # seconds to hold full lock while turning
TURN_ANGLE         = 35    # steering angle during a 90° turn (= MAX_STEERING)
TURN_SPEED         = 22    # speed during turn execution

# Default intersection behaviour: 'left', 'right', or 'straight'
INTERSECTION_DEFAULT = 'straight'

# ── Recovery (line lost) ──
RECOVERY_BACK_SPEED = 20
RECOVERY_BACK_TIME  = 0.4   # seconds to back up
RECOVERY_SWEEP      = [-25, 25, -35, 35]   # angles to sweep while searching
RECOVERY_SWEEP_DWELL = 0.25  # seconds at each sweep angle
MAX_RECOVERY_ATTEMPTS = 5    # give up and stop after this many failures


# ─────────────────────────────────────────────
# PHASE 1 — CALIBRATION
# ─────────────────────────────────────────────

def _apply_offset(offsets):
    px.dir_cali_val       = offsets[0]
    px.cam_pan_cali_val   = offsets[1]
    px.cam_tilt_cali_val  = offsets[2]
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)


def _test_movement():
    print("Testing movement — make sure the robot has room!")
    for angle in (0, 10, -10):
        px.set_dir_servo_angle(angle)
        px.forward(30)
        sleep(1)
    px.stop(); sleep(0.5)
    for angle in (0, 10, -10):
        px.set_dir_servo_angle(angle)
        px.backward(30)
        sleep(1)
    px.stop()


def _show_cal(servo, offsets):
    names = ["Steering", "Camera Pan", "Camera Tilt"]
    print("\033[H\033[J", end="")
    print("========= PiCar-X CALIBRATION TOOL =========")
    print(f"  Selected : {names[servo]}")
    print(f"  Offsets  : steering={offsets[0]}  pan={offsets[1]}  tilt={offsets[2]}")
    print()
    print("  1/2/3  → select servo")
    print("  W/D    → increase offset    S/A → decrease offset")
    print("  R      → test movement")
    print("  SPACE  → save & start line-following")
    print("  CTRL+C → exit")
    print("=============================================")


def run_calibration():
    """Interactive calibration. Returns when user presses SPACE."""
    servo   = 0
    offsets = [0, 0, 0]
    step    = 1

    _show_cal(servo, offsets)

    while True:
        key = readchar.readkey().lower()

        if key in ('1', '2', '3'):
            servo = int(key) - 1
            _show_cal(servo, offsets)

        elif key in ('w', 'd'):
            offsets[servo] += step
            _apply_offset(offsets)
            _show_cal(servo, offsets)

        elif key in ('s', 'a'):
            offsets[servo] -= step
            _apply_offset(offsets)
            _show_cal(servo, offsets)

        elif key == 'r':
            _test_movement()
            _show_cal(servo, offsets)

        elif key == readchar.key.SPACE:
            print("\nSaving calibration…")
            px.dir_servo_calibrate(offsets[0])
            px.cam_pan_servo_calibrate(offsets[1])
            px.cam_tilt_servo_calibrate(offsets[2])
            print("Calibration saved. Starting line-following in 2 seconds…")
            sleep(2)
            return   # → Phase 2

        elif key == readchar.key.CTRL_C:
            print("Exiting.")
            raise SystemExit


# ─────────────────────────────────────────────
# PHASE 2 — LINE FOLLOWING HELPERS
# ─────────────────────────────────────────────

def read_sensors():
    """Return raw grayscale values (L, C, R)."""
    return px.get_grayscale_data()   # list of 3 ints


def sensors_on_line(data):
    """True for each sensor that is over a dark line."""
    return [v < DARK_THRESHOLD for v in data]


def compute_error(data):
    """
    Weighted position error in roughly [-1, +1].
    Negative = line is to the left, positive = to the right.
    Returns None if no sensor sees the line.
    """
    on_line = sensors_on_line(data)
    if not any(on_line):
        return None
    weighted = sum(w for w, s in zip(SENSOR_WEIGHTS, on_line) if s)
    count    = sum(on_line)
    return weighted / count


def is_intersection(data):
    """All three sensors dark simultaneously → intersection or T-junction."""
    return all(v < DARK_THRESHOLD for v in data)


# ─────────────────────────────────────────────
# INTERSECTION HANDLER
# ─────────────────────────────────────────────

def handle_intersection(direction=INTERSECTION_DEFAULT):
    """
    Called when an intersection is confirmed.
    Drives a short distance forward (to centre the robot on the crossing),
    then executes the requested turn.

    direction: 'left', 'right', or 'straight'
    """
    print(f"[INTERSECTION] → {direction}")

    # 1. Drive forward through the intersection box
    px.set_dir_servo_angle(0)
    px.forward(BASE_SPEED)
    sleep(TURN_DRIVE_TIME)

    if direction == 'straight':
        # Just keep going — line-following will re-acquire
        return

    # 2. Apply full steering lock and drive around the corner
    turn_angle = -TURN_ANGLE if direction == 'left' else TURN_ANGLE
    px.set_dir_servo_angle(turn_angle)
    px.forward(TURN_SPEED)
    sleep(TURN_EXECUTE_TIME)

    # 3. Straighten up — the main loop re-acquires the line
    px.set_dir_servo_angle(0)


# ─────────────────────────────────────────────
# 90-DEGREE TURN (sharp corner on the track)
# ─────────────────────────────────────────────

def execute_90_turn(direction):
    """
    Hard 90° turn used when the robot loses the line on one side —
    indicating a sharp corner rather than a gradual curve.

    direction: 'left' (line last seen left) or 'right'
    """
    print(f"[90° TURN] → {direction}")
    turn_angle = -TURN_ANGLE if direction == 'left' else TURN_ANGLE
    px.set_dir_servo_angle(turn_angle)
    px.forward(TURN_SPEED)
    sleep(TURN_EXECUTE_TIME)
    px.set_dir_servo_angle(0)


# ─────────────────────────────────────────────
# LINE-LOSS RECOVERY
# ─────────────────────────────────────────────

def recover(last_error):
    """
    Called when no sensor sees the line.
    1. If the previous error was large (line was far to one side),
       attempt a 90° turn in that direction first.
    2. Otherwise back up and sweep the front sensors.
    Returns True if the line was re-acquired, False if not.
    """
    # Sharp-corner attempt: if error was strongly one-sided, try a 90° turn
    if last_error is not None and abs(last_error) > 0.6:
        direction = 'right' if last_error > 0 else 'left'
        execute_90_turn(direction)
        data = read_sensors()
        if any(sensors_on_line(data)):
            print("[RECOVERY] Line re-acquired after 90° turn.")
            return True

    # General sweep recovery
    print("[RECOVERY] Backing up and sweeping…")
    px.set_dir_servo_angle(0)
    px.backward(RECOVERY_BACK_SPEED)
    sleep(RECOVERY_BACK_TIME)
    px.stop()

    for angle in RECOVERY_SWEEP:
        px.set_dir_servo_angle(angle)
        sleep(RECOVERY_SWEEP_DWELL)
        data = read_sensors()
        if any(sensors_on_line(data)):
            print(f"[RECOVERY] Line found at sweep angle {angle}°.")
            px.set_dir_servo_angle(0)
            return True

    px.set_dir_servo_angle(0)
    return False


# ─────────────────────────────────────────────
# PHASE 2 — MAIN LINE-FOLLOWING LOOP
# ─────────────────────────────────────────────

def run_line_following():
    print("\033[H\033[J", end="")
    print("========= LINE FOLLOWING ACTIVE =========")
    print("  CTRL+C to stop")
    print("=========================================\n")

    # PID state
    integral   = 0.0
    prev_error = 0.0
    last_error = None   # most recent valid error (for recovery direction)

    # Intersection debounce
    intersection_start = None

    # Recovery counter
    recovery_attempts = 0

    px.set_dir_servo_angle(0)
    px.forward(BASE_SPEED)

    try:
        while True:
            data = read_sensors()

            # ── Intersection detection ────────────────────────
            if is_intersection(data):
                if intersection_start is None:
                    intersection_start = time()
                elif time() - intersection_start >= INTERSECTION_TIME:
                    # Confirmed intersection
                    px.stop()
                    handle_intersection(INTERSECTION_DEFAULT)
                    intersection_start = None
                    integral   = 0.0   # reset integrator after a turn
                    prev_error = 0.0
                    px.forward(BASE_SPEED)
                continue   # skip normal PID while in intersection window

            else:
                intersection_start = None   # reset debounce if sensors split

            # ── Compute PID error ─────────────────────────────
            error = compute_error(data)

            if error is None:
                # No sensor sees the line → recovery
                px.stop()
                recovered = recover(last_error)
                recovery_attempts += 1

                if not recovered or recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                    print("[STOP] Line lost — could not recover.")
                    break

                px.forward(BASE_SPEED)
                integral   = 0.0
                prev_error = 0.0
                continue

            # Line acquired — reset recovery counter
            recovery_attempts = 0
            last_error = error

            # ── PID calculation ───────────────────────────────
            integral  += error
            derivative = error - prev_error
            prev_error = error

            # Clamp integral to prevent wind-up
            integral = max(-10.0, min(10.0, integral))

            raw_steering = KP * error + KI * integral + KD * derivative
            steering = max(-MAX_STEERING, min(MAX_STEERING, raw_steering))

            # ── Dynamic speed ─────────────────────────────────
            speed = BASE_SPEED * (1.0 - SPEED_REDUCTION * abs(error))
            speed = max(MIN_SPEED, speed)

            # ── Apply ─────────────────────────────────────────
            px.set_dir_servo_angle(steering)
            px.forward(speed)

            # Short cycle delay — keep CPU headroom on the Pi
            sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        px.stop()
        px.set_dir_servo_angle(0)


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    try:
        run_calibration()   # Phase 1
        run_line_following() # Phase 2
    finally:
        px.stop()