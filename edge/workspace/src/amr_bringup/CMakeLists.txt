cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rotate_image SHARED src/rotate_image.cpp)
target_compile_features(rotate_image PUBLIC cxx_std_17)
target_link_libraries(rotate_image PUBLIC
  rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} tf2_ros::tf2_ros)
rclcpp_components_register_nodes(rotate_image "amr_bringup::RotateImage")
install(TARGETS rotate_image
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config rviz worlds textures
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/move_arm_to_standby.py scripts/arm_bringup.sh scripts/depth_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
