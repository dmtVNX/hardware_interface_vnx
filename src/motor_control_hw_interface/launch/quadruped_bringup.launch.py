import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #  Get package directories
    hw_interface_pkg = get_package_share_directory('motor_control_hw_interface')
    
    #  URDF file (you need to create this based on your robot)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("your_robot_description"),  # Your URDF package
            "urdf",
            "quadruped.urdf.xacro"
        ]),
    ])
    
    robot_description = {"robot_description": robot_description_content}

    #  Controller config
    robot_controllers = PathJoinSubstitution([
        hw_interface_pkg,
        "config",
        "quadruped_controllers.yaml",
    ])

    #  Controller Manager Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    #  Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    #  Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    #  Joint Trajectory Controller Spawner
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    #  Delay controller spawners
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    #  Start CAN bridges (from motor_control package)
    start_can_bridges = ExecuteProcess(
        cmd=['bash', '-c', 
             'source ' + os.path.join(get_package_share_directory('motor_control'), 'scripts/start_can_bridges.sh')],
        output='screen'
    )

    return LaunchDescription([
        start_can_bridges,
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
    ])