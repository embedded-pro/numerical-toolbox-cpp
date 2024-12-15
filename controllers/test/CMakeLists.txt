add_executable(dsp.controllers_test)
emil_build_for(dsp.controllers_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.controllers_test)

target_link_libraries(dsp.controllers_test PUBLIC
    gmock_main
    dsp.controllers
)

target_sources(dsp.controllers_test PRIVATE
    TestPid.cpp
)
