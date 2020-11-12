# CMake generated Testfile for 
# Source directory: /home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/integration
# Build directory: /home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/build/nav2_3d_static_layer/test/integration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(static3d_tests "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/build/nav2_3d_static_layer/test_results/nav2_3d_static_layer/static3d_tests.xml" "--package-name" "nav2_3d_static_layer" "--generate-result-on-success" "--env" "TEST_MAP=/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/map/TenByTen.yaml" "TEST_LAUNCH_DIR=/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/test_launch_files" "TEST_EXECUTABLE=/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/build/nav2_3d_static_layer/test/integration/static3d_tests_exec" "--command" "/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/integration/costmap_tests_launch.py")
set_tests_properties(static3d_tests PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/build/nav2_3d_static_layer/test/integration" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/integration/CMakeLists.txt;12;ament_add_test;/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/integration/CMakeLists.txt;0;")
subdirs("../../gtest")
