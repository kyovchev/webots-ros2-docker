import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_simulation')
    with open(os.path.join(package_dir, 'resource', 'my_robot.urdf'), 'r') as f:
        robot_description = f.read()

    with open(os.path.join(package_dir, 'resource', 'object_detector.urdf'), 'r') as f:
        object_detector_description = f.read()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        ros2_supervisor=True
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description},
        ],

        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    object_detector = WebotsController(
        robot_name='object_detector',
        parameters=[
            {'robot_description': object_detector_description},
        ],

        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        object_detector,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
