add_executable(dsp.filters.passive_test)
emil_build_for(dsp.filters.passive_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.filters.passive_test)

target_link_libraries(dsp.filters.passive_test PUBLIC
    gmock_main
    dsp.filters.passive
    dsp.math_test_helper
)

target_sources(dsp.filters.passive_test PRIVATE
    TestIir.cpp
)
