from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    can_iface = LaunchConfiguration('can_iface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_iface', default_value='can0',
            description='SocketCAN interface name'
        ),
        
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                get_package_share_directory('ros2_socketcan')
                + '/launch/socket_can_bridge.launch.xml'
            ),
            launch_arguments={
                'interface': can_iface,
            }.items(),
        ),

        Node(
            package='motor_control',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            remappings=[
                ('from_can_bus', '/from_can_bus'),
                ('to_can_bus',   '/to_can_bus')
            ],
        ),
    ])