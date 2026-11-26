"""rover1.launch.py - bring up rover1: robot_state_publisher, ros2_control, diff_drive_controller.

  ros2 launch rover1_bringup rover1.launch.py                      # real robot on /dev/rover1_llc
  ros2 launch rover1_bringup rover1.launch.py device:=/dev/ttyUSB0
  ros2 launch rover1_bringup rover1.launch.py use_mock_hardware:=true   # no robot needed
  ros2 launch rover1_bringup rover1.launch.py teleop:=joy                # also start gamepad teleop

Drive with /cmd_vel (geometry_msgs/Twist). Odometry: /odom and TF odom -> base_footprint.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover1_bringup')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    device = LaunchConfiguration('device')
    teleop = LaunchConfiguration('teleop')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', PathJoinSubstitution([pkg, 'urdf', 'rover1.urdf.xacro']),
                ' use_mock_hardware:=', use_mock_hardware,
                ' device:=', device,
            ]),
            value_type=str,
        )
    }
    controllers = PathJoinSubstitution([pkg, 'config', 'rover1_controllers.yaml'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # Humble: the controller manager takes robot_description as a parameter.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_drive_controller/odom', '/odom'),
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )
    # start the drive controller once the joint state broadcaster is up
    diff_drive_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[diff_drive])
    )

    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'teleop_joy.launch.py'])),
        condition=IfCondition(PythonExpression(["'", teleop, "' == 'joy'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock_components/GenericSystem instead of the ESP32'),
        DeclareLaunchArgument('device', default_value='/dev/rover1_llc',
                              description='ESP32 serial port (udev/99-rover1.rules creates /dev/rover1_llc)'),
        DeclareLaunchArgument('teleop', default_value='none',
                              description="'joy' starts gamepad teleop; for the keyboard run "
                                          "'ros2 run teleop_twist_keyboard teleop_twist_keyboard' in a terminal"),
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        diff_drive_after_jsb,
        teleop_joy,
    ])
