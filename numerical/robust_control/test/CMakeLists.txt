add_executable(numerical.robust_control_test)
emil_build_for(numerical.robust_control_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.robust_control_test)

target_link_libraries(numerical.robust_control_test PUBLIC
    gmock_main
    numerical.robust_control
)

numerical_add_qemu_test(numerical.robust_control_test)

target_sources(numerical.robust_control_test PRIVATE
    TestActiveDisturbanceRejection.cpp
    TestDisturbanceObserver.cpp
    TestHInfinityStateFeedback.cpp
    TestSlidingModeControl.cpp
)
