import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'two_wheel_robot.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path]
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'],
            output='screen',
            on_exit=LogInfo(msg='joint_state_publisher_gui exited (not installed?)')
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
