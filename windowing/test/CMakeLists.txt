add_executable(dsp.windowing_test)
emil_build_for(dsp.windowing_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.windowing_test)

target_link_libraries(dsp.windowing_test PUBLIC
    gmock_main
    dsp.windowing
)

target_sources(dsp.windowing_test PRIVATE
    TestWindowing.cpp
)
