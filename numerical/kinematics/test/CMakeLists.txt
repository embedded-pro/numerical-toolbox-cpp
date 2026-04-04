add_executable(numerical.kinematics_test)
emil_build_for(numerical.kinematics_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.kinematics_test)

target_link_libraries(numerical.kinematics_test PUBLIC
    gmock_main
    numerical.kinematics
)

target_sources(numerical.kinematics_test PRIVATE
    TestForwardKinematics.cpp
)
