add_executable(toolbox.controllers_test)
emil_build_for(toolbox.controllers_test BOOL EMIL_BUILD_TESTS)
emil_add_test(toolbox.controllers_test)

target_link_libraries(toolbox.controllers_test PUBLIC
    gmock_main
    control_toolbox.pid
)

target_sources(toolbox.controllers_test PRIVATE
    TestPid.cpp
)
