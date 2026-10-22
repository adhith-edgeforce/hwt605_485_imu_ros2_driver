from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='230400'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
    ]
    node = Node(
        package='hwt605_driver_fast',
        executable='hwt605_node',
        name='hwt605_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )
    return LaunchDescription(args + [node])
