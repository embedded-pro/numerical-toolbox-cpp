add_executable(numerical.motion_profiles_test)
emil_build_for(numerical.motion_profiles_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.motion_profiles_test)

target_link_libraries(numerical.motion_profiles_test PUBLIC
    gmock_main
    numerical.motion_profiles
)

numerical_link_qemu_runtime(numerical.motion_profiles_test)

target_sources(numerical.motion_profiles_test PRIVATE
    TestPolynomialProfile.cpp
    TestSCurveProfile.cpp
    TestTrapezoidalProfile.cpp
)
