cmake_minimum_required(VERSION 3.5)
project(nav2_3d_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/nav2_3d_costmap.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_3d_costmap.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()    