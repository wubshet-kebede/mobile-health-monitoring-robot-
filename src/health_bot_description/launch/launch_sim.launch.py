import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'health_bot_description'
    
    # 1. Path Resolution
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'health_hospital.world')
    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'bridge_params.yaml')

    # 2. Robot State Publisher (RSP)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 4. Spawn the Robot (Increased Z for safety)
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'health_bot', '-z', '0.05'],
        output='screen'
    )

    # 5. The ROS-GZ Bridge (The Missing Link)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True
        }],
        output="screen"
    )

    # 6. Return all actions to be executed
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge  # <--- Added to the list!
    ])
