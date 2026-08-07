add_executable(numerical.filters.active_test)
emil_build_for(numerical.filters.active_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.filters.active_test)

target_link_libraries(numerical.filters.active_test PUBLIC
    gmock_main
    numerical.filters.active
    numerical.math_test_helper
)

numerical_add_qemu_test(numerical.filters.active_test)

target_sources(numerical.filters.active_test PRIVATE
    TestAhrsMadgwickMahony.cpp
    TestAlphaBetaFilter.cpp
    TestComplementaryFilter.cpp
    TestExtendedKalmanFilter.cpp
    TestKalmanFilter.cpp
    TestKalmanSmoother.cpp
    TestSquareRootKalmanFilter.cpp
    TestUnscentedKalmanFilter.cpp
)
