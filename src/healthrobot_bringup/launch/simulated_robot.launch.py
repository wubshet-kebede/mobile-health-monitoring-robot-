import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to other packages
    description_pkg = get_package_share_directory('health_bot_description')
    control_pkg = get_package_share_directory('healthbot_ros2_control')

    # Include robot description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'launch_sim.launch.py')
        )
    )

    # Include ros2_control launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'healthbot_control.launch.py')
        )
    )

    # Return combined launch description
    return LaunchDescription([
        description_launch,
        control_launch
    ])
