from picarx import Picarx
from time import sleep

px = Picarx()

def test_movement(offset):
    print(f"\nTesting offset: {offset}")
    px.set_dir_servo_angle(offset)

    # Forward straight
    px.forward(30)
    sleep(1)

    # Forward right
    px.set_dir_servo_angle(offset + 10)
    px.forward(30)
    sleep(1)

    # Forward left
    px.set_dir_servo_angle(offset - 10)
    px.forward(30)
    sleep(1)

    # Stop briefly
    px.stop()
    sleep(0.5)

    # Backward straight
    px.set_dir_servo_angle(offset)
    px.backward(30)
    sleep(1)

    # Backward right
    px.set_dir_servo_angle(offset + 10)
    px.backward(30)
    sleep(1)

    # Backward left
    px.set_dir_servo_angle(offset - 10)
    px.backward(30)
    sleep(1)

    px.stop()


def calibrate_steering():
    offset = 0
    minAngle = -15
    maxAngle = 15

    print("Starting wheel calibration...")

    for _ in range(6):  # limit iterations
        test_movement(offset)

        direction = input("Drift? (l=left, r=right, s=straight): ")

        if direction == 'l':
            print("Adjusting right")
            minAngle = offset
        elif direction == 'r':
            print("Adjusting left")
            maxAngle = offset
        elif direction == 's':
            print("Calibration complete")
            break

        offset = (minAngle + maxAngle) / 2
        print(f"New offset: {offset}")

    print(f"Final steering offset: {offset}")

    with open("steering_calibration.txt", "w") as f:
        f.write(str(offset))

    return offset


if __name__ == "__main__":
    calibrate_steering()