cmake_minimum_required(VERSION 3.5)
project(nav2_3d_costmap_plugin)

#set
set(lib_name ${PROJECT_NAME}_core)
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
#set(PCL_DIR /usr/local/share/pcl-1.11/PCLConfig.cmake)


#find_packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filter)

set(dep_pkgs

        rclcpp
        nav2_costmap_2d
        pluginlib
        pcl_conversions
        geometry_msgs
        )


#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#add_executable process func



# === Build ===

add_library(${lib_name} SHARED
#            src/nav2_3d_costmap.cpp
            src/nav2_3d_buffer.cpp
#            src/nav2_3d_grid.cpp
            src/process.cpp
#        include/nav2_3d_costmap_plugin/nav2_3d_buffer.hpp
#        include/nav2_3d_costmap_plugin/nav2_3d_costmap.hpp
#        include/nav2_3d_costmap_plugin/nav2_3d_grid.hpp
        )
#add dependencies to project
add_executable(nav2_3d_costmap_plugin src/process.cpp)
target_link_libraries(nav2_3d_costmap_plugin ${lib_name} ${PCL_LIBRARIES})
ament_target_dependencies(nav2_3d_costmap_plugin ${dep_pkgs})

include_directories(
        include
        ${PCL_INCLUDE_DIRS})
#target_link_libraries(nav2_3d_costmap_plugin ${PCL_LIBRARIES})
# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_3d_costmap.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()    