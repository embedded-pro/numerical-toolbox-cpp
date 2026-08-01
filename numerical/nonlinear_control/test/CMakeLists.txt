add_executable(numerical.nonlinear_control_test)
emil_build_for(numerical.nonlinear_control_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.nonlinear_control_test)

target_link_libraries(numerical.nonlinear_control_test PUBLIC
    gmock_main
    numerical.nonlinear_control
)

target_sources(numerical.nonlinear_control_test PRIVATE
    TestFeedbackLinearization.cpp
)
