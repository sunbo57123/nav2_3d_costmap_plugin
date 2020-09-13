cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(nav2_3d_costmap_plugin)

#set
list(APPEND CMAKE_MODULE_PATH "/home/sun/openvdb/cmake")
set(lib_name ${PROJECT_NAME}_core)
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

#find_packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.11 REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    pcl_conversions
    geometry_msgs
    )

add_library(${lib_name} SHARED
            src/nav2_3d_costmap.cpp
            src/nav2_3d_buffer.cpp
#            src/process.cpp
            src/nav2_3d_grid.cpp
            )

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_executable(nav2_3d_costmap_plugin
#        src/nav2_3d_buffer.cpp
#        src/nav2_3d_costmap.cpp
#        src/nav2_3d_grid.cpp)

#target_link_libraries(nav2_3d_costmap_plugin ${PCL_LIBRARIES})
#target_link_libraries(nav2_3d_costmap_plugin OpenVDB::openvdb)
target_link_libraries(${lib_name} ${PCL_LIBRARIES} OpenVDB::openvdb)
ament_target_dependencies(${lib_name} ${dep_pkgs})

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_3d_costmap.xml)
ament_package()