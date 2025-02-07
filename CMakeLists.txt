cmake_minimum_required(VERSION 3.8)
project(opencv_ros_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# Define the executable
add_executable(camera_publisher src/camera_publisher.cpp)

# Link libraries
target_link_libraries(camera_publisher
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

# Install targets
install(TARGETS camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
