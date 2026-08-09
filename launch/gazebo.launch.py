from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tulip_gazebo')
    
    world_file = os.path.join(pkg_share, 'building_robot.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'robot.rviz')
    
    return LaunchDescription([
        # Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # ArUco Detector
        Node(
            package='tulip_gazebo',
            executable='aruco_detector',
            output='screen'
        ),
        
        # Robot Localizer
        Node(
            package='tulip_gazebo',
            executable='robot_localizer',
            output='screen'
        ),
        
        # Target Detector
        Node(
            package='tulip_gazebo',
            executable='target_detector',
            output='screen'
        ),
        
        # Movement Node
        Node(
            package='tulip_gazebo',
            executable='movement',
            output='screen'
        ),
    ])
