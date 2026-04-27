from picarx import Picarx
from time import sleep

px = Picarx()

def calibrate_sensors():
    print("=== SENSOR CALIBRATION ===")
    print("Place sensors over WHITE surface")
    input("Press Enter to capture WHITE values...")

    white_vals = px.get_grayscale_data()
    print(f"White values: {white_vals}")

    sleep(1)

    print("\nPlace sensors over BLACK line")
    input("Press Enter to capture BLACK values...")

    black_vals = px.get_grayscale_data()
    print(f"Black values: {black_vals}")

    # Compute threshold (midpoint)
    thresholds = [
        (white_vals[i] + black_vals[i]) / 2
        for i in range(3)
    ]

    print("\n=== CALIBRATION RESULTS ===")
    print(f"Thresholds: {thresholds}")

    # Save to file
    with open("sensor_thresholds.txt", "w") as f:
        f.write(",".join(map(str, thresholds)))

    print("Saved to sensor_thresholds.txt")

    return thresholds


if __name__ == "__main__":
    calibrate_sensors()