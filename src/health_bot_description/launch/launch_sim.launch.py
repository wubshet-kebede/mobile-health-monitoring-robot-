import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'health_bot_description'
    world_file_name = 'health_hospital.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', world_file_name)
    # 1. Include the Robot State Publisher (The one we made earlier)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo launch file
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    #     )]), launch_arguments={'gz_args': '-r empty.sdf'}.items()
    # )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )
    # 3. Run the spawner node to put the robot in Gazebo
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'health_bot',
                                   '-z', '0.1'],
                        output='screen')

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
