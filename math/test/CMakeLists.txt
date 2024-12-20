add_executable(dsp.math_test)
emil_build_for(dsp.math_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.math_test)

target_link_libraries(dsp.math_test PUBLIC
    gmock_main
    dsp.math
)

target_sources(dsp.math_test PRIVATE
    TestQNumber.cpp
)
