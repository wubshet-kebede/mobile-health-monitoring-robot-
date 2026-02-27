import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to  modular YAML
    config = os.path.join(
        get_package_share_directory("healthbot_ros2_control"),
        "config",
        "healthbot_controllers.yaml"
    )

    # Controller Manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config],
        output="screen"
    )

    # Spawner for Joint State Broadcaster
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # Spawner for Diff Drive Controller
    spawner_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["healthbot_controller"],
        output="screen"
    )

    # Spawner for Simple Velocity Controller
    spawner_vel = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller"],
        output="screen"
    )

    return LaunchDescription([
        controller_manager,
        spawner_jsb,
        spawner_diff,
        spawner_vel
    ])
