import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nidar_gazebo = get_package_share_directory('nidar_gazebo')
    pkg_nidar_description = get_package_share_directory('nidar_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_nidar_gazebo, 'worlds', 'rescueswarm_flood_zone.world')
    bridge_config = os.path.join(pkg_nidar_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_nidar_description, 'urdf', 'rescueswarm_drone.urdf.xacro')

    robot_desc_d1 = Command(['xacro ', xacro_file, ' drone_name:=drone_1'])
    robot_desc_d2 = Command(['xacro ', xacro_file, ' drone_name:=drone_2'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_drone_1 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'drone_1',
            '-string', robot_desc_d1,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ]
    )

    spawn_drone_2 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'drone_2',
            '-string', robot_desc_d2,
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.5'
        ]
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        spawn_drone_1,
        spawn_drone_2,
        gz_bridge
    ])
