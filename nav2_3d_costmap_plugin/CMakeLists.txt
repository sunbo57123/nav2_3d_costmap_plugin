cmake_minimum_required(VERSION 3.5)
project(simple_plugin)

set(lib_name ${PROJECT_NAME}_core)
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.5 REQUIRED)
find_package(octomap REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    pcl_conversions
    geometry_msgs
        )

add_library(${lib_name} SHARED
        src/simple_plugin.cpp)
include_directories(
        include
        ${PCL_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS ${lib_name}
        DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d simple_plugin.xml)
target_link_libraries(${lib_name} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
ament_target_dependencies(${lib_name} ${dep_pkgs})

ament_package()
