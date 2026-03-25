add_executable(numerical.analysis_test)
emil_build_for(numerical.analysis_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.analysis_test)

target_link_libraries(numerical.analysis_test PUBLIC
    gmock_main
    numerical.analysis
    numerical.math_test_helper
)

target_sources(numerical.analysis_test PRIVATE
    TestDiscreteCosineTransform.cpp
    TestFastFourierTransformRadix2Impl.cpp
    TestPowerDensitySpectrum.cpp
)
