from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    declared_param_list = []

    declared_param_list.append(DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Visualize robot in RViZ"))

    declared_param_list.append(DeclareLaunchArgument(
        "description_package",
        default_value="quadrotor_sim",
        description="ROS2 package name"))

    declared_param_list.append(DeclareLaunchArgument(
            "description_file",
            default_value="quad_base.xacro",
            description="URDF/XACRO description file with the robot."))
    
    use_rviz = LaunchConfiguration('use_rviz')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content,value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    # launch_gazebo_arg = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([FindPackageShare('ros_ign_gazebo'), '/launch', '/ign_gazebo.launch.py']),
    #     launch_arguments={
    #     # 'ign_args' : [FindPackageShare(description_package), "world", "empty.world"]
    #     }.items(),
    #     )


    # gazebo_node = Node(
    #         package='ros_ign_gazebo',
    #         executable='create',
    #         output='screen',
    #         arguments=["-param", "robot_description", "-name", "quadrotor"]
    # )

    nodes_to_start = [
        # launch_gazebo_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        # gazebo_node
    ]

    return LaunchDescription(declared_param_list + nodes_to_start)