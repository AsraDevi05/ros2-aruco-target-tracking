#!/bin/bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
source ~/Documents/RE402_ROS/install/setup.bash

cat > /tmp/bridge_config.yaml << 'BEOF'
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odom"
  gz_topic_name: "/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
BEOF

echo "Starting Gazebo..."
gz sim -r tulip.urdf &
sleep 5

echo "Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/tmp/bridge_config.yaml &
sleep 2

echo "Starting cam node..."
ros2 run tulip_gazebo cam &
sleep 1

echo "Starting cam_localization node..."
ros2 run tulip_gazebo cam_localization &
sleep 1

echo "Starting aruco_detector node..."
ros2 run tulip_gazebo aruco_detector &
sleep 1

echo "Starting target_detector node..."
ros2 run tulip_gazebo target_detector &
sleep 1

echo "Starting robot_localizer node..."
ros2 run tulip_gazebo robot_localizer &
sleep 1

echo "Starting movement node..."
ros2 run tulip_gazebo movement &

wait
