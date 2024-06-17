cmake_minimum_required(VERSION 3.8)
project(camera)

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
find_package(message REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/EventBatch.msg"
	DEPENDENCIES std_msgs
	)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs message cv_bridge sensor_msgs)
target_link_libraries(talker ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(talkerfast src/talkerfast.cpp)
ament_target_dependencies(talkerfast rclcpp std_msgs message cv_bridge sensor_msgs)
target_link_libraries(talkerfast ${OpenCV_INCLUDE_DIRS} dv::processing)


add_executable(event src/event.cpp)
ament_target_dependencies(event rclcpp std_msgs message cv_bridge sensor_msgs)
target_link_libraries(event ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(frame src/frame.cpp)
ament_target_dependencies(frame rclcpp std_msgs message cv_bridge sensor_msgs)
target_link_libraries(frame ${OpenCV_INCLUDE_DIRS} dv::processing)

add_executable(control src/control.cpp)
ament_target_dependencies(control rclcpp std_msgs message cv_bridge sensor_msgs geometry_msgs)
target_link_libraries(control ${OpenCV_INCLUDE_DIRS} dv::processing)



install(TARGETS talker talkerfast event frame control
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
