# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_name = 'health_bot_description'
    
#     # Path Resolution
#     pkg_robot_share = get_package_share_directory(pkg_name)
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     bridge_params = os.path.join(pkg_robot_share, 'config', 'bridge_params.yaml')

#     # World file now inside health_bot_description
#     world_file = os.path.join(pkg_robot_share, 'worlds', 'hospital.world')

#     # Gazebo resource path points only to health_bot_description
#     set_gz_resource_path = SetEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=[
#             os.path.join(pkg_robot_share, 'models'),
#             ':',
#             os.path.join(pkg_robot_share, 'urdf'),
#             ':',
#             pkg_robot_share
#         ]
#     )
#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(pkg_robot_share, 'launch', 'rsp.launch.py')
#         ]),
#         launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
#         ),
#         launch_arguments={
#             'gz_args': [f'-r -v 4 ', world_file],
#             'on_exit_shutdown': 'true'
#         }.items()
#     )


#     spawn_robot = Node(
#         package='ros_gz_sim', 
#         executable='create',
#         arguments=[
#             '-name', 'health_bot',
#             '-topic', 'robot_description', 
#             '-x', '0.5', 
#             '-y', '1.5', 
#             '-z', '0.0',
#             '-Y', '0.0'
#         ],
#         output='screen'
#     )
    
#     # ROS-GZ Bridge
#     bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         parameters=[{
#             'config_file': bridge_params,
#             'use_sim_time': True
#         }],
#         output="screen"
#     )

#     slam = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(pkg_robot_share, 'launch', 'slam.launch.py')
#         ]),
#         launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     # Return all actions to be executed
#     return LaunchDescription([
#         set_gz_resource_path,
#         rsp,
#         gazebo,
#         spawn_robot,
#         bridge,
#         slam
#     ])
import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    health_bot_description = get_package_share_directory("health_bot_description")
    bridge_params = os.path.join(health_bot_description, 'config', 'bridge_params.yaml')

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                health_bot_description, "urdf", "healthbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="hospital")

    world_path = PathJoinSubstitution([
            health_bot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(health_bot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("health_bot_description"), 'models')
    model_path += pathsep + os.path.join(get_package_share_directory("health_bot_description"), 'meshes')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " is_sim:=true"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "health_bot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True
        }],
        output="screen"
    )
    controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[os.path.join(
        get_package_share_directory("health_bot_description"),
        "config",
        "healthbot_controllers.yaml"
    )],
    output="screen"
)
    spawner_jsb = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen" 
    )
    spawner_diff = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["healthbot_controller"],
        output="screen"
    )
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        controller_manager,
        spawner_jsb,
        spawner_diff
    ])