import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Setup paths
    pkg_hospital_world = get_package_share_directory('health_hospital_world')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 2. Path to your SDF/World file
    world_file = os.path.join(pkg_hospital_world, 'worlds', 'hospital.world')

    # 3. CRITICAL: Tell Harmonic where to find your medical models
    # This replaces the old GAZEBO_MODEL_PATH
    models_path = os.path.join(pkg_hospital_world, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path]
    )

    # 4. Include the Gazebo Harmonic launch file
    # 'gz_args' replaces the old server/client split. 
    # '-r' means run immediately, '-v 4' is for verbose logs
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [f'-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo
    ])
