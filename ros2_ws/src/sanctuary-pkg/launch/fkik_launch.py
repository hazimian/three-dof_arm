from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('theta1', default_value='0.0'),
        DeclareLaunchArgument('theta2', default_value='0.0'),
        DeclareLaunchArgument('theta3', default_value='0.0'),
        Node(
            package='sanctuary-pkg',
            executable='fk_node',
            name='talker',
            output='screen',
            parameters=[{
                'theta1': LaunchConfiguration('theta1'),
                'theta2': LaunchConfiguration('theta2'),
                'theta3': LaunchConfiguration('theta3')
            }]
        ),
        Node(
            package='sanctuary-pkg',
            executable='ik_node',
            name='listener',
            output='screen'
        ),
    ])
