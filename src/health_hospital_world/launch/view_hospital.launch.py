import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Path to your NEW package and the hospital launch file we just made
    pkg_hospital_world = get_package_share_directory('health_hospital_world')
    hospital_launch_path = os.path.join(pkg_hospital_world, 'launch', 'hospital.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hospital_launch_path),
            # In Harmonic, if you want NO gui, you usually pass '-s' in gz_args
            # For now, we just call the main launch file
            launch_arguments={}.items()
        )
    ])
