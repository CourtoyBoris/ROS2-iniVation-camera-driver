cmake_minimum_required(VERSION 3.8)
project(camera_stereo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

file(MAKE_DIRECTORY "/tmp/binarydeb/ros-galactic-zstd-vendor-0.9.2/.obj-x86_64-linux-gnu/zstd_vendor_install/include")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dv-processing REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_stereo REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/EventBatch.msg"
	DEPENDENCIES std_msgs
	)

add_executable(frame src/frame.cpp)
ament_target_dependencies(frame rclcpp std_msgs message cv_bridge sensor_msgs)
target_link_libraries(frame ${OpenCV_INCLUDE_DIRS} dv::processing)


add_executable(event_visualizer src/event_visualizer.cpp)
ament_target_dependencies(event_visualizer rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(event_visualizer ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(talker_duo src/talker_duo.cpp)
ament_target_dependencies(talker_duo rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(talker_duo ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(talker_eros src/talker_eros.cpp)
ament_target_dependencies(talker_eros rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(talker_eros ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(eros src/eros.cpp)
ament_target_dependencies(eros rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(eros ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(eros_visualizer src/eros_visualizer.cpp)
ament_target_dependencies(eros_visualizer rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(eros_visualizer ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(talker_tos src/talker_tos.cpp)
ament_target_dependencies(talker_tos rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(talker_tos ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(blur_filter src/blur_filter.cpp)
ament_target_dependencies(blur_filter rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(blur_filter ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(eros_alt src/eros_alt.cpp)
ament_target_dependencies(eros_alt rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(eros_alt ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(winner src/winner.cpp)
ament_target_dependencies(winner rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(winner ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(high_pass_filter src/high_pass_filter.cpp)
ament_target_dependencies(high_pass_filter rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(high_pass_filter ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(accumulator src/accumulator.cpp)
ament_target_dependencies(accumulator rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(accumulator ${OpenCV_INCLUDE_DIRS} dv::processing)


install(TARGETS frame talker_duo event_visualizer eros eros_visualizer talker_eros talker_tos blur_filter eros_alt winner high_pass_filter accumulator
  DESTINATION lib/${PROJECT_NAME})
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
