from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "my_robot",
        package_name="my_robot_moveit_config"
    ).to_moveit_configs()

    demo_launch = generate_demo_launch(moveit_config)

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        demo_launch
    ])
