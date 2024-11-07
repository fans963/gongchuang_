cmake_minimum_required(VERSION 3.8)
project(gongchuang)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# find\_package(libusb-1.0 REQUIRED)
# include\_directories(${libusb-1.0\_INCLUDE\_DIRS})
# target\_link\_libraries(your\_target ${libusb-1.0\_LIBRARIES})

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

add_executable( 
  gongchuang 
  src/main.cpp
)

target_include_directories(gongchuang PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${libusb-1.0\_INCLUDE\_DIRS})

ament_target_dependencies( 
  gongchuang
  rclcpp
  std_msgs 
  OpenCV
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} -lusb-1.0)
install(
  TARGETS
  gongchuang
  DESTINATION
  lib/${PROJECT_NAME}
)


ament_package()

