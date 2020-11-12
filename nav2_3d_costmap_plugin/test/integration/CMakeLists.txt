
ament_add_gtest_executable(static3d_tests_exec
        static3d_tests.cpp
        )
ament_target_dependencies(static3d_tests_exec
        ${dep_pkgs}
        )
target_link_libraries(static3d_tests_exec
        nav2_3d_static_layer_core
        )
message("here")
ament_add_test(static3d_tests
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        ENV
        TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
        TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
        TEST_EXECUTABLE=$<TARGET_FILE:static3d_tests_exec>
        )
message("here")