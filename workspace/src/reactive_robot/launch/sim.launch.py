import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('reactive_robot')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # Paths to both the world AND the robot files
    world_path = os.path.join(worlds_dir, 'reactive_test.sdf')
    robot_path = os.path.join(worlds_dir, 'reactive_bot.sdf')

    set_env_var = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        worlds_dir
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r "{world_path}"'}.items()
    )

    SAFE_ZONES = [
        [-7.0, -4.5, -5.0,  4.0],  # Left Zone
        [ 2.5,  5.0, -5.0,  4.0],  # Right Zone
        [-7.0,  5.0, -8.0, -6.0],  # Top Zone (Y < -5)
        [-7.0,  5.0,  5.0,  7.0]   # Bottom Zone (Y > 4)
    ]

    #Pick a random zone and spawn the robot
    chosen_zone = random.choice(SAFE_ZONES)
    spawn_x = str(random.uniform(chosen_zone[0], chosen_zone[1]))
    spawn_y = str(random.uniform(chosen_zone[2], chosen_zone[3]))
    spawn_yaw = str(random.uniform(-3.14159, 3.14159))

    # Node to inject the robot into Gazebo at the random coordinates
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'reactive_test',
            '-file', robot_path,
            '-name', 'reactive_bot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.08',
            '-Y', spawn_yaw
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    radar_node = Node(
        package='reactive_robot',
        executable='radar_node',
        output='screen'
    )

    decision_node = Node(
        package='reactive_robot',
        executable='decision_node',
        output='screen'
    )

    movement_node = Node(
        package='reactive_robot',
        executable='movement_node',
        output='screen'
    )

    return LaunchDescription([
        set_env_var,
        gz_sim,
        spawn_robot,
        bridge,
        radar_node,
        decision_node,
        movement_node
    ])