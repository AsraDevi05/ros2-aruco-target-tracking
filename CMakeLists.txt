cmake_minimum_required(VERSION 3.8)
project(tulip_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)

add_executable(waypoint src/waypoint.cpp)
ament_target_dependencies(waypoint
rclcpp geometry_msgs tf2)

add_executable(movement src/movement.cpp)
ament_target_dependencies(movement
rclcpp geometry_msgs std_msgs tf2)

add_executable(cam_localization src/cam_localization.cpp)
ament_target_dependencies(cam_localization
rclcpp geometry_msgs sensor_msgs tf2)

add_executable(cam src/cam.cpp)
ament_target_dependencies(cam
rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(target_detector src/target_detector.cpp)
ament_target_dependencies(target_detector
rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

# --- Node baru: deteksi ArUco robot dari kamera overhead ---
add_executable(aruco_detector src/deteksi_aruco.cpp)
ament_target_dependencies(aruco_detector
rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV tf2)

# --- Node baru: fusion ArUco + Odom -> /pose ---
add_executable(robot_localizer src/robot_localizer.cpp)
ament_target_dependencies(robot_localizer
rclcpp geometry_msgs nav_msgs tf2)

add_executable(udp_bridge_node src/udp_bridge_node.cpp)
ament_target_dependencies(udp_bridge_node rclcpp geometry_msgs sensor_msgs)

find_package(OpenCV REQUIRED)
# OpenCV aruco biasanya sudah termasuk, tapi jika tidak:
target_link_libraries(target_detector
  ${OpenCV_LIBS}
)

# --- target_bridge ---
add_executable(target_bridge src/target_bridge.cpp)
ament_target_dependencies(target_bridge rclcpp geometry_msgs std_msgs tf2)

# add_executable(ball_detector src/ball_detector.cpp)
# ament_target_dependencies(ball_detector rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

install(TARGETS cam DESTINATION lib/${PROJECT_NAME})
install(TARGETS cam_localization DESTINATION lib/${PROJECT_NAME})
install(TARGETS target_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS obstacle_avoidance DESTINATION lib/${PROJECT_NAME})
install(TARGETS waypoint DESTINATION lib/${PROJECT_NAME})
install(TARGETS movement DESTINATION lib/${PROJECT_NAME})
install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS robot_localizer DESTINATION lib/${PROJECT_NAME})
install(TARGETS udp_bridge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS target_bridge DESTINATION lib/${PROJECT_NAME})
# install(TARGETS ball_detector DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

# # --- target_bridge: jembatan target_detector -> movement_node ---
# add_executable(target_bridge src/target_bridge.cpp)
# ament_target_dependencies(target_bridge
#   rclcpp geometry_msgs std_msgs tf2)
# install(TARGETS target_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config worlds
    DESTINATION share/${PROJECT_NAME}
)
