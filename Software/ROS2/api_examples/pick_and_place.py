import time
import requests

# URL of your FastAPI ROS 2 bridge (note the updated port 8001 from your script)
BASE_URL = "http://localhost:8001"

# Gripper positions (integers, same range as the bridge / gripper node: 0..100).
# Adjust these to match which end of the range is open / closed on your gripper.
GRIPPER_OPEN = 20
GRIPPER_CLOSED = 70

# The servo has no feedback, so after a gripper command we just wait this long
# for it to finish moving (seconds).
GRIPPER_SETTLE_TIME = 0.1

# Define a list of target coordinates and orientations to visit.
# Optional key "gripper": a gripper position to apply AFTER the arm reaches
# that waypoint. Waypoints without it leave the gripper unchanged.
waypoints = [
    {"x": 0.14, "y": 0.00, "z": 0.10, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False,
        "gripper": GRIPPER_OPEN},

    {"x": 0.17, "y": 0.01, "z": 0.07, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False},
    {"x": 0.17, "y": 0.01, "z": 0.00, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False,
        "gripper": GRIPPER_CLOSED},
    {"x": 0.17, "y": 0.01, "z": 0.07, "roll": 3.14,
     "pitch": 0.00, "yaw": 0.00, "cartesian_path": False},

    {"x": 0.1, "y": -0.17, "z": 0.07, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False},
    {"x": 0.1, "y": -0.17, "z": 0.00, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False,
        "gripper": GRIPPER_OPEN},
    {"x": 0.1, "y": -0.17, "z": 0.07, "roll": 3.14,
        "pitch": 0.00, "yaw": 0.00, "cartesian_path": False},

    {"x": 0.14, "y": 0.00, "z": 0.10, "roll": 3.14,
     "pitch": 0.00, "yaw": 0.00, "cartesian_path": False},
]


def send_pose(pose):
    """Sends a single pose command to the FastAPI server."""
    # The "gripper" key is handled separately, so don't send it with the pose
    pose_only = {k: v for k, v in pose.items() if k != "gripper"}
    try:
        response = requests.post(f"{BASE_URL}/api/move", json=pose_only)
        if response.status_code == 200:
            print(
                f"Successfully sent waypoint: x={pose['x']}, y={pose['y']}, z={pose['z']}")
        else:
            print(f"Failed to send pose: {response.json()}")
    except requests.exceptions.ConnectionError:
        print("Error: Could not connect to the FastAPI server. Is it running?")


def send_gripper(position):
    """Sends a gripper position command (integer) to the FastAPI server."""
    try:
        response = requests.post(
            f"{BASE_URL}/api/gripper", json={"position": int(position)})
        if response.status_code == 200:
            print(f"Successfully sent gripper command: {position}")
            return True
        print(f"Failed to send gripper command: {response.json()}")
    except requests.exceptions.ConnectionError:
        print("Error: Could not connect to the FastAPI server. Is it running?")
    return False


def get_gripper_position():
    """Returns the last gripper position commanded on /gripper_position (no real feedback)."""
    try:
        response = requests.get(f"{BASE_URL}/api/gripper")
        if response.status_code == 200:
            return response.json().get("position")
    except requests.exceptions.ConnectionError:
        pass
    return None


def set_gripper_and_wait(position):
    """Sends a gripper command, then waits for the servo to finish moving."""
    if send_gripper(position):
        time.sleep(GRIPPER_SETTLE_TIME)
        print(f"Gripper last commanded position: {get_gripper_position()}")


def check_if_moving():
    """Queries the FastAPI server to see if the robot is currently moving."""
    try:
        response = requests.get(f"{BASE_URL}/api/status")
        if response.status_code == 200:
            return response.json().get("is_moving", False)
    except requests.exceptions.ConnectionError:
        pass
    return False


def get_current_pose():
    """Fetches the latest forward kinematics pose of the end-effector."""
    try:
        response = requests.get(f"{BASE_URL}/api/ee_pose")
        if response.status_code == 200:
            return response.json()
    except requests.exceptions.ConnectionError:
        pass
    return None


def main():
    print("Starting waypoint navigation script...")

    # Start with the gripper open
    print("\n--- Opening gripper ---")
    set_gripper_and_wait(GRIPPER_OPEN)

    for i, target in enumerate(waypoints):
        print(f"\n--- Sending Waypoint {i + 1} of {len(waypoints)} ---")

        # 1. Send the coordinate command
        send_pose(target)

        # # Print final reached pose
        # final_pose = get_current_pose()
        # if final_pose:
        #     print(
        #         f"Waypoint {i + 1} reached! Actual EE Position: x={final_pose.get('x', 0):.4f}, y={final_pose.get('y', 0):.4f}, z={final_pose.get('z', 0):.4f}")
        # else:
        #     print(f"Waypoint {i + 1} reached!")

        # 1. Gripper action for this waypoint, if any (arm is stopped at this point)
        if "gripper" in target:
            print(f"Gripper action at waypoint {i + 1}...")
            set_gripper_and_wait(target["gripper"])

        # Pause between waypoints
        time.sleep(0.1)

    print("\nAll waypoints completed successfully!")


if __name__ == "__main__":
    main()
