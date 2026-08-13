import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nidar_gazebo = get_package_share_directory('nidar_gazebo')
    pkg_nidar_description = get_package_share_directory('nidar_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_nidar_gazebo, 'worlds', 'airmouse_maze.world')
    bridge_config = os.path.join(pkg_nidar_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_nidar_description, 'urdf', 'airmouse_drone.urdf.xacro')

    robot_desc = Command(['xacro ', xacro_file])

    # 1. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Spawn Drone Model in Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'airmouse_drone',
            '-topic', 'robot_description',
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.2'
        ]
    )

    # 4. ROS GZ Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_drone,
        gz_bridge
    ])
