import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    esp_ip_arg = DeclareLaunchArgument(
        "esp_ip", default_value="172.20.10.4",
        description="ESP32 IP address"
    )

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        parameters=[{
            "video_device": "/dev/video2",
            "image_width": 640,
            "image_height": 480,
            "pixel_format": "mjpeg",
            "framerate": 30.0,
        }],
        output="screen",
    )

    aruco = Node(
        package="tulip_gazebo",
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[{
            "field_size": 6.0,
            "field_offset": 3.0,
            "dictionary": "DICT_7X7_50",
            "target_marker_id": -1,
        }],
        output="screen",
    )

    target = Node(
        package="tulip_gazebo",
        executable="target_detector",
        name="target_detector",
        parameters=[{
            "field_size": 6.0,
            "field_offset": 3.0,
        }],
        output="screen",
    )

    localizer = Node(
        package="tulip_gazebo",
        executable="robot_localizer",
        name="robot_localizer",
        parameters=[{"aruco_timeout_sec": 2.0}],
        output="screen",
    )

    movement = Node(
        package="tulip_gazebo",
        executable="movement",
        name="movement_node",
        parameters=[{
            "max_linear_speed": 0.3,
            "max_angular_speed": 1.0,
            "kp_linear": 0.6,
            "kp_angular": 1.8,
            "arrive_tolerance": 0.15,
            "resume_tolerance": 0.30,
            "yaw_tolerance": 0.05,
            "rotate_start_threshold": 0.4,
            "control_hz": 20.0,
        }],
        output="screen",
    )

    bridge = Node(
        package="tulip_bridge",
        executable="bridge",
        name="tulip_bridge",
        parameters=[{
            "esp_ip": LaunchConfiguration("esp_ip"),
            "esp_port": 8888,
            "cmd_timeout": 0.5,
        }],
        output="screen",
    )

    return LaunchDescription([
        esp_ip_arg,
        camera,
        aruco,
        target,
        localizer,
        movement,
        bridge,
    ])
