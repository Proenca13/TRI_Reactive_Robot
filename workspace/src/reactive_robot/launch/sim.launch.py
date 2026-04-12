import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('reactive_robot')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # Paths to both worlds AND the robot files
    default_world_path = os.path.join(worlds_dir, 'reactive_test.sdf')
    old_5_world_path = os.path.join(worlds_dir, 'old_5_world.sdf')
    robot_path = os.path.join(worlds_dir, 'reactive_bot.sdf')

    # Declare the old_5 launch argument
    old_5_arg = DeclareLaunchArgument(
        'old_5',
        default_value='false',
        description='Set to true to use the old_5 configuration'
    )

    set_env_var = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        worlds_dir
    )

    # Base Gazebo launch source
    gz_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    # Launch Default World (Runs ONLY if old_5:=false)
    gz_sim_default = IncludeLaunchDescription(
        gz_launch_source,
        launch_arguments={'gz_args': f'-r "{default_world_path}"'}.items(),
        condition=UnlessCondition(LaunchConfiguration('old_5'))
    )

    # Launch Old 5 World (Runs ONLY if old_5:=true)
    gz_sim_old_5 = IncludeLaunchDescription(
        gz_launch_source,
        launch_arguments={'gz_args': f'-r "{old_5_world_path}"'}.items(),
        condition=IfCondition(LaunchConfiguration('old_5'))
    )

    SAFE_ZONES = [
        [-7.0, -4.5, -5.0,  4.0],  # Left Zone
        [ 2.5,  5.0, -5.0,  4.0],  # Right Zone
        [-7.0,  5.0, -8.0, -6.0],  # Top Zone (Y < -5)
        [-7.0,  5.0,  5.0,  7.0]   # Bottom Zone (Y > 4)
    ]

    # Pick a random zone and spawn the robot
    chosen_zone = random.choice(SAFE_ZONES)
    spawn_x = str(random.uniform(chosen_zone[0], chosen_zone[1]))
    spawn_y = str(random.uniform(chosen_zone[2], chosen_zone[3]))
    spawn_yaw = str(random.uniform(-3.14159, 3.14159))

    # Node to inject the robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
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
        old_5_arg,
        set_env_var,
        gz_sim_default,
        gz_sim_old_5,
        spawn_robot,
        bridge,
        radar_node,
        decision_node,
        movement_node
    ])