add_executable(dsp.analysis_test)
emil_build_for(dsp.analysis_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.analysis_test)

target_link_libraries(dsp.analysis_test PUBLIC
    gmock_main
    dsp.analysis
    dsp.math_test_helper
)

target_sources(dsp.analysis_test PRIVATE
    TestDiscreteCosineTransform.cpp
    TestFastFourierTransformRadix2Impl.cpp
    TestPowerDensitySpectrum.cpp
)
