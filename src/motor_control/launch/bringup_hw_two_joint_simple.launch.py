from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    can_iface = LaunchConfiguration('can_iface')
    urdf_file = LaunchConfiguration('urdf')

    # Generate robot_description from xacro at runtime
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_iface', default_value='can0'),
        DeclareLaunchArgument(
            'urdf',
            default_value=os.path.join(
                get_package_share_directory('motor_control'),
                'config',
                'two_joint_hw_simple.urdf.xacro'
            )
        ),

        # SocketCAN bridge (XML launch from ros2_socketcan)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros2_socketcan'),
                    'launch',
                    'socket_can_bridge.launch.xml'
                )
            ),
            launch_arguments={'interface': can_iface}.items(),
        ),

        TimerAction(
            period=0.1,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[
                        {'robot_description': robot_description},
                        os.path.join(
                            get_package_share_directory('motor_control'),
                            'config',
                            'jgp_hw_simple.yaml'
                        ),
                    ],
                    output='both'
                ),
                Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
                Node(package='controller_manager', executable='spawner', arguments=['pos_group']),
            ]
        ),
    ])
