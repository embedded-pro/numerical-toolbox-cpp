add_executable(dsp.filters.active_test)
emil_build_for(dsp.filters.active_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.filters.active_test)

target_link_libraries(dsp.filters.active_test PUBLIC
    gmock_main
    dsp.filters.active
)

target_sources(dsp.filters.active_test PRIVATE
    TestKalmanFilter.cpp
)
