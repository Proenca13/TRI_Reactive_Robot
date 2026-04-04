import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('reactive_robot')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    world_path = os.path.join(worlds_dir, 'reactive_test.sdf')

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
        launch_arguments={'gz_args': f'-r "{world_path}" --render-engine ogre'}.items()
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

    return LaunchDescription([
        set_env_var,
        gz_sim,
        bridge,
        radar_node,
        decision_node
    ])