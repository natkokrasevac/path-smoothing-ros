cmake_minimum_required(VERSION 3.10.2)
project(path_smoothing_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(statistics_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(cubic_spline_interpolator
  src/cubic_spline_interpolator.cpp
)
ament_target_dependencies(cubic_spline_interpolator rclcpp geometry_msgs nav_msgs statistics_msgs std_msgs rosgraph_msgs)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp geometry_msgs nav_msgs statistics_msgs std_msgs rosgraph_msgs)
target_link_libraries(demo cubic_spline_interpolator)



install(TARGETS
  demo
  cubic_spline_interpolator
  DESTINATION lib/${PROJECT_NAME}/
)


install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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