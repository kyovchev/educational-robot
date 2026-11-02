import time
import requests

# URL of your FastAPI ROS 2 bridge (note the updated port 8001 from your script)
BASE_URL = "http://localhost:8001"


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


def main():
    send_pose({"x": 0.0851, "y": 0.109, "z": 0.1879, "roll": 0.00,
               "pitch": 3.14/4, "yaw": 3.14, "cartesian_path": False})

    send_pose({"x": 0.0851, "y": 0.109, "z": 0.1879, "roll": 0.00,
                   "pitch": 3.14/4, "yaw": 3.14, "cartesian_path": False})
if __name__ == "__main__":
    main()
