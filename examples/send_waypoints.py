import time
import requests

# URL of your FastAPI ROS 2 bridge (note the updated port 8001 from your script)
BASE_URL = "http://localhost:8001"

# Define a list of target coordinates and orientations to visit
waypoints = [
    {"x": 0.0, "y": 0.12, "z": 0.05, "roll": 3.14,
        "pitch": 0.0, "yaw": 0.0, "cartesian_path": False},
    {"x": 0.05, "y": 0.12, "z": 0.05, "roll": 3.14,
        "pitch": 0.0, "yaw": 0.0, "cartesian_path": False},
    {"x": 0.0, "y": 0.12, "z": 0.02, "roll": 3.14,
        "pitch": 0.0, "yaw": 0.0, "cartesian_path": False},
    {"x": -0.05, "y": 0.12, "z": 0.05, "roll": 3.14,
        "pitch": 0.0, "yaw": 0.0, "cartesian_path": False},
    {"x": 0.0, "y": 0.12, "z": 0.05, "roll": 3.14,
        "pitch": 0.0, "yaw": 0.0, "cartesian_path": False},
]


def send_pose(pose):
    """Sends a single pose command to the FastAPI server."""
    try:
        response = requests.post(f"{BASE_URL}/api/move", json=pose)
        if response.status_code == 200:
            print(
                f"Successfully sent waypoint: x={pose['x']}, y={pose['y']}, z={pose['z']}")
        else:
            print(f"Failed to send pose: {response.json()}")
    except requests.exceptions.ConnectionError:
        print("Error: Could not connect to the FastAPI server. Is it running?")


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

    for i, target in enumerate(waypoints):
        print(f"\n--- Sending Waypoint {i + 1} of {len(waypoints)} ---")

        # 1. Send the coordinate command
        send_pose(target)

        # 2. Short pause to allow the robot to register motion and start moving
        time.sleep(1.0)

        # 3. Wait while the robot is moving and print live pose feedback
        print("Waiting for robot to reach destination...")
        while check_if_moving():
            pose = get_current_pose()
            if pose:
                print(
                    f"  -> Current EE Pose: x={pose.get('x', 0):.4f}, y={pose.get('y', 0):.4f}, z={pose.get('z', 0):.4f}", end="\r")
            time.sleep(0.5)  # Poll every half second

        print("\n")
        # Print final reached pose
        final_pose = get_current_pose()
        if final_pose:
            print(
                f"Waypoint {i + 1} reached! Actual EE Position: x={final_pose.get('x', 0):.4f}, y={final_pose.get('y', 0):.4f}, z={final_pose.get('z', 0):.4f}")
        else:
            print(f"Waypoint {i + 1} reached!")

        # Optional pause between waypoints
        time.sleep(1.0)

    print("\nAll waypoints completed successfully!")


if __name__ == "__main__":
    main()
