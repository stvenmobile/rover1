"""teleop_joy.launch.py - gamepad -> /cmd_vel (joy + teleop_twist_joy). Hold the enable button to drive."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('rover1_bringup'), 'config', 'teleop_joy.yaml'])
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[config]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[config]),  # publishes /cmd_vel
    ])
