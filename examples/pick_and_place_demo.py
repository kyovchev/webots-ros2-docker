import sys
import time
import requests

BASE_URL = "http://localhost:8001"          # FastAPI ROS 2 bridge
SCENE_URL = "http://localhost:8002"         # Webots scene REST API
OBJECT_NAME = "TARGET_BOX"                  # DEF name in the .wbt

# ---------------------------------------------------------------------------
# Scene configuration: adjust to your Webots world (meters)
# ---------------------------------------------------------------------------
# Robot base_link position in the Webots world frame. The scene API works in
# world coordinates, the robot API in base_link coordinates.
ROBOT_BASE_WORLD = (0.05, -0.05, 0.01)

ORIENTATION = {"roll": 3.14, "pitch": 0.0, "yaw": 0.0}  # tool pointing down

HOME = (0.0, 0.12, 0.10)   # start/end pose, must be collision free

PICK_XY = (0.05, 0.12)     # object position (base_link frame)
PLACE_XY = (-0.05, 0.12)   # drop position (base_link frame)

BOX_HEIGHT = 0.01          # object height

# Height of the surfaces the objects rest on, in the WORLD frame. They are
# converted to the base_link frame below using the robot base height.
PICK_SURFACE_WORLD_Z = 0.0
PLACE_SURFACE_WORLD_Z = 0.0

# Distance from the tool0 origin to the suction cup tip along the tool axis.
# If tool0 is not at the cup tip, set this, or the cup stops short of the object.
TCP_ABOVE_CUP_TIP = 0.0

PICK_SURFACE_Z = PICK_SURFACE_WORLD_Z - \
    ROBOT_BASE_WORLD[2]     # base_link frame
PLACE_SURFACE_Z = PLACE_SURFACE_WORLD_Z - ROBOT_BASE_WORLD[2]

# Derived tool0 heights (base_link frame)
OBJECT_TOP_Z = PICK_SURFACE_Z + BOX_HEIGHT + \
    TCP_ABOVE_CUP_TIP            # cup touches the top
PLACE_Z = PLACE_SURFACE_Z + BOX_HEIGHT + 0.005 + \
    TCP_ABOVE_CUP_TIP        # release with a 5 mm drop
SAFE_Z = max(OBJECT_TOP_Z, PLACE_Z) + \
    0.08                                # travel height

SUCTION_SETTLE_S = 0.8
RELEASE_SETTLE_S = 0.5
MOVE_START_TIMEOUT_S = 3.0
MOVE_DONE_TIMEOUT_S = 30.0


# ---------------------------------------------------------------------------
# Scene API (Webots scene_manipulator)
# ---------------------------------------------------------------------------
def to_world(x, y, z):
    bx, by, bz = ROBOT_BASE_WORLD
    return [bx + x, by + y, bz + z]


def scene_get_pose(name):
    r = requests.get(f"{SCENE_URL}/objects/{name}/pose", timeout=5)
    r.raise_for_status()
    return r.json()


def scene_set_pose(name, **pose):
    r = requests.put(f"{SCENE_URL}/objects/{name}/pose", json=pose, timeout=5)
    r.raise_for_status()
    return r.json()


def reset_object():
    """Put the box at the pick position, upright, centered above the table."""
    pos = to_world(PICK_XY[0], PICK_XY[1], PICK_SURFACE_Z + BOX_HEIGHT / 2)
    scene_set_pose(OBJECT_NAME, position=pos, rpy=[0, 0, 0])
    print(f"Object {OBJECT_NAME} placed at world {pos}")
    time.sleep(0.5)  # let physics settle


def report_result():
    final = scene_get_pose(OBJECT_NAME)["position"]
    target = to_world(PLACE_XY[0], PLACE_XY[1],
                      PLACE_SURFACE_Z + BOX_HEIGHT / 2)
    err_xy = sum((a - b) ** 2 for a, b in zip(final[:2], target[:2])) ** 0.5
    err_z = final[2] - target[2]
    print(f"Object final position: {[round(v, 3) for v in final]}")
    print(
        f"Placement error: xy {err_xy * 1000:.1f} mm, z {err_z * 1000:+.1f} mm")


# ---------------------------------------------------------------------------
# Robot API helpers
# ---------------------------------------------------------------------------
def api_get(path):
    r = requests.get(f"{BASE_URL}{path}", timeout=2)
    r.raise_for_status()
    return r.json()


def api_post(path, payload):
    r = requests.post(f"{BASE_URL}{path}", json=payload, timeout=2)
    r.raise_for_status()
    return r.json()


def is_moving():
    return api_get("/api/status").get("is_moving", False)


def move_to(x, y, z, cartesian=False):
    """Send a pose and block until the robot has started and finished moving."""
    api_post("/api/move", {"x": x, "y": y, "z": z, **ORIENTATION,
                           "cartesian_path": cartesian})
    print(f"  move -> x={x:.3f} y={y:.3f} z={z:.3f}"
          f"{' (cartesian)' if cartesian else ''}")

    t0 = time.time()
    started = False
    while time.time() - t0 < MOVE_START_TIMEOUT_S:
        if is_moving():
            started = True
            break
        time.sleep(0.05)
    if not started:
        time.sleep(0.3)
        return

    t0 = time.time()
    while is_moving():
        if time.time() - t0 > MOVE_DONE_TIMEOUT_S:
            raise TimeoutError("Robot did not finish moving in time")
        time.sleep(0.1)
    time.sleep(0.2)


def gripper(on):
    api_post("/api/gripper", {"on": on})
    print(f"  gripper {'ON' if on else 'OFF'}")


def object_attached():
    return api_get("/api/gripper").get("object_attached", False)


# ---------------------------------------------------------------------------
# Pick and place
# ---------------------------------------------------------------------------
def pick(x, y):
    print(f"Picking at ({x:.3f}, {y:.3f})")
    move_to(x, y, SAFE_Z)
    gripper(True)
    move_to(x, y, OBJECT_TOP_Z, cartesian=True)
    time.sleep(SUCTION_SETTLE_S)
    move_to(x, y, SAFE_Z, cartesian=True)

    if not object_attached():
        gripper(False)
        raise RuntimeError(
            "Pick failed: no object detected on the suction cup")
    print("  object attached")


def place(x, y):
    print(f"Placing at ({x:.3f}, {y:.3f})")
    move_to(x, y, SAFE_Z)
    move_to(x, y, PLACE_Z, cartesian=True)
    gripper(False)
    time.sleep(RELEASE_SETTLE_S)
    move_to(x, y, SAFE_Z, cartesian=True)

    if object_attached():
        print("  warning: presence still reports an object after release")


def main():
    try:
        api_get("/api/status")
    except requests.exceptions.RequestException:
        print("Could not reach the FastAPI bridge. Is it running?")
        sys.exit(1)
    try:
        scene_get_pose(OBJECT_NAME)
    except requests.exceptions.RequestException as e:
        print(
            f"Could not use the scene API at {SCENE_URL} for {OBJECT_NAME}: {e}")
        sys.exit(1)

    print("Starting pick and place demo")
    gripper(False)
    move_to(*HOME)
    reset_object()

    try:
        pick(*PICK_XY)
        place(*PLACE_XY)
        move_to(*HOME)
        time.sleep(0.5)
        report_result()
        print("Demo complete")
    except (RuntimeError, TimeoutError) as e:
        print(f"Aborting: {e}")
        gripper(False)
        move_to(*HOME)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Interrupted, releasing suction")
        gripper(False)
        sys.exit(130)


if __name__ == "__main__":
    main()
