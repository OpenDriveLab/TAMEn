cmake_minimum_required(VERSION 3.20)
project(robohub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(sensor_msgs REQUIRED)  # Ensure sensor_msgs is found
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(k1_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include the Protobuf header directories
include_directories(${PROTOBUF_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Include path for JAKA SDK headers
# include_directories(${CMAKE_SOURCE_DIR}/include)

## Add the JAKA SDK library path
link_directories(${CMAKE_SOURCE_DIR}/out/shared)
#

## Define the executable target
# add_executable(jaka_driver_node dual_arm_driver.cpp common.h common.cpp timespec.h timespec.cpp zucpos2.h kine.h)
add_executable(jaka_driver_node dual_arm_driver_template.cpp common.h common.cpp timespec.h timespec.cpp)
ament_target_dependencies(jaka_driver_node rclcpp std_msgs sensor_msgs geometry_msgs std_srvs k1_msgs tf2)
#
#target_include_directories(jaka_driver_node PRIVATE /opt/ros/humble)
# target_link_libraries(jaka_driver_node rclcpp std_msgs)
target_include_directories(jaka_driver_node PRIVATE ${CMAKE_SOURCE_DIR}/inc)
# ament_target_dependencies(jaka_driver_node jakaAPI_2_3_0_7 pthread ${PROTOBUF_LIBRARIES})
#
## Link rclcpp and other required libraries
target_link_libraries(jaka_driver_node jakaAPI_2_3_0_DUAL_4 pthread ${PROTOBUF_LIBRARIES})

# Set RPATH so the executable can find colocated libraries at runtime
set_target_properties(jaka_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Export dependencies automatically
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs std_srvs k1_msgs tf2)

# Installation rules
install(TARGETS jaka_driver_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/out/shared/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so")

ament_package()






