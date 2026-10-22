from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 1. Include Webots simulation
    webots_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('webots_simulation').find(
                'webots_simulation'), 'launch', 'simulation.launch.py')
        ])
    )

    # 2. Delay the entire MoveIt demo launch by 8 seconds
    delayed_moveit_demo = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(FindPackageShare('my_robot_moveit_config').find(
                        'my_robot_moveit_config'), 'launch', 'demo.launch.py')
                ])
            )
        ]
    )
    # 3. Delay the controller spawners or ensure they load after Webots connects
    # (If demo.launch.py auto-spawns, you might want to look into demo.launch.py
    # to pass an argument like `db:=false` or disable its internal spawners if needed).

    # 4. Delay starting your commander and pick_and_place nodes until everything is stable
    delayed_commander = TimerAction(
        period=15.0,  # Wait 10 seconds for Webots and MoveIt to fully spin up
        actions=[
            Node(
                package='my_robot_commander_cpp',
                executable='commander',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    delayed_pick_and_place = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='my_robot_pick_and_place',
                executable='pick_and_place_server',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'port': 9090}
        ]
    )

    ros_web_service = Node(
        package='webots_simulation',
        executable='ros_web_service',
        name='ros_web_service_node',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        webots_sim,
        delayed_moveit_demo,
        delayed_commander,
        delayed_pick_and_place,
        web_video_server_node,
        ros_web_service
    ])
