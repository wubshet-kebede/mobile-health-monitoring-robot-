import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    pkg_name = 'health_bot_description'
    pkg_hospital = 'health_hospital_world'
    
    #  Path Resolution
    pkg_robot_share = get_package_share_directory(pkg_name)
    pkg_hospital_share = get_package_share_directory(pkg_hospital)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'bridge_params.yaml')
    world_file = os.path.join(pkg_hospital_share, 'worlds', 'hospital.world')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_hospital_share, 'models'),
            ':',
            os.path.join(pkg_robot_share, 'urdf'), 
            ':',
            pkg_robot_share 
        ]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [f'-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-name', 'health_bot',
            '-topic', 'robot_description', 
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '1.5' 
        ],
        output='screen'
    )
    
    #  The ROS-GZ Bridge 
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True
        }],
        output="screen"
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'slam.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    #  Return all actions to be executed
    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        rsp,
        spawn_robot,
        bridge,
        slam
        
    ])
