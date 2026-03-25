add_executable(numerical.control_analysis_test)
emil_build_for(numerical.control_analysis_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.control_analysis_test)

target_link_libraries(numerical.control_analysis_test PUBLIC
    gmock_main
    numerical.control_analysis
    numerical.math_test_helper
)

target_sources(numerical.control_analysis_test PRIVATE
    TestFrequencyResponse.cpp
    TestRootLocus.cpp
)
