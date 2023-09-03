from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    base_rviz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('quadrotor_sim'), '/launch', '/rviz_launch.py']),
        launch_arguments={
        'ign_args' : "-r -v 3 empty.sdf"
        }.items(),
        )

    launch_gazebo_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
        'ign_args' : "-r -v 3 empty.sdf"
        }.items(),
        )

    gazebo_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=["-topic", "/robot_description", "-name", "quadrotor","-world","empty"]
    )

    nodes_to_start = [
        base_rviz_sim,
        launch_gazebo_arg,
        gazebo_node
    ]

    return LaunchDescription(nodes_to_start)