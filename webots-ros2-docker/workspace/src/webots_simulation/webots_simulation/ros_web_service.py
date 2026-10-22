from fastapi import FastAPI, HTTPException
import uvicorn
import threading
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import PoseCommand
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

app = FastAPI(title="ROS 2 Web Service Bridge")

# Global variables for ROS communication and state caching
ros_node = None
pose_publisher = None
is_robot_moving = False
latest_ee_pose = {"x": 0.0, "y": 0.0, "z": 0.0,
                  "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


class RosWebServiceNode(Node):
    def __init__(self):
        super().__init__('ros_web_service_node')

        # Publishers and Subscribers
        self.pose_pub = self.create_publisher(PoseCommand, '/pose_command', 10)
        self.moving_sub = self.create_subscription(
            Bool, '/is_moving', self.moving_callback, 10)

        # TF Listener for Forward Kinematics (End-Effector Pose)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically update the cached EE pose (e.g., at 20Hz)
        self.timer = self.create_timer(0.05, self.update_ee_pose)

    def moving_callback(self, msg: Bool):
        global is_robot_moving
        is_robot_moving = msg.data

    def update_ee_pose(self):
        global latest_ee_pose
        try:
            # Replace 'base_link' and 'tool0' with your robot's exact frame names if different
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time())

            # Extract position
            latest_ee_pose["x"] = trans.transform.translation.x
            latest_ee_pose["y"] = trans.transform.translation.y
            latest_ee_pose["z"] = trans.transform.translation.z

            # Optional: Extract orientation quaternion (or convert to RPY if preferred)
            latest_ee_pose["qx"] = trans.transform.rotation.x
            latest_ee_pose["qy"] = trans.transform.rotation.y
            latest_ee_pose["qz"] = trans.transform.rotation.z
            latest_ee_pose["qw"] = trans.transform.rotation.w
        except Exception:
            # TF might not be ready immediately on startup; ignore silently until available
            pass


def ros_spin_thread():
    global ros_node, pose_publisher
    rclpy.init()
    ros_node = RosWebServiceNode()
    pose_publisher = ros_node.pose_pub

    try:
        rclpy.spin(ros_node)
    except Exception:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

# --- API Endpoints ---


@app.post("/api/move")
def send_pose_command(pose: dict):
    global pose_publisher
    if pose_publisher is None:
        raise HTTPException(
            status_code=503, detail="ROS 2 node not initialized yet.")

    try:
        msg = PoseCommand()
        msg.x = float(pose.get("x", 0.0))
        msg.y = float(pose.get("y", 0.0))
        msg.z = float(pose.get("z", 0.0))
        msg.roll = float(pose.get("roll", 0.0))
        msg.pitch = float(pose.get("pitch", 0.0))
        msg.yaw = float(pose.get("yaw", 0.0))
        msg.cartesian_path = bool(pose.get("cartesian_path", False))

        pose_publisher.publish(msg)
        return {"status": "success", "message": "Pose command published successfully."}
    except Exception as e:
        raise HTTPException(
            status_code=400, detail=f"Failed to publish message: {str(e)}")


@app.get("/api/status")
def get_robot_status():
    global is_robot_moving
    return {"is_moving": is_robot_moving}


@app.get("/api/ee_pose")
def get_end_effector_pose():
    """Returns the latest forward kinematics pose of the robot's end-effector from TF."""
    global latest_ee_pose
    return latest_ee_pose


def main():
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8001)


if __name__ == '__main__':
    main()
