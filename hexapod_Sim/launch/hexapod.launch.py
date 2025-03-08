from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

def generate_launch_description():
    home_directory = os.getenv('HOME')
    if not home_directory:
        raise RuntimeError("Environment variable 'HOME' is not set. Unable to locate the home directory.")

    urdf_file_path = os.path.join(
        home_directory,
        'Documents/ros2_ws/src/hexapod_Sim/urdf/Hexapod.xacro.urdf'
    )

    if not os.path.isfile(urdf_file_path):
        raise RuntimeError(f"URDF file not found at: {urdf_file_path}")
    print(urdf_file_path)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path]),
                value_type=str
            )
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    converter = Node(
        package='hexapod_Sim',
        executable='HexaTopicConverterFull',
        name='HexaTopicConverterFull',
        output='screen'
    )

    control = Node(
        package='hexapod_control',
        executable='test_node',
        name='test_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        converter,
        control
        # joint_state_publisher_gui_node
    ])