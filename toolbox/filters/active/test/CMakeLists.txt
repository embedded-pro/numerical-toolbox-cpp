add_executable(numerical.filters.active_test)
emil_build_for(numerical.filters.active_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.filters.active_test)

target_link_libraries(numerical.filters.active_test PUBLIC
    gmock_main
    numerical.filters.active
)

target_sources(numerical.filters.active_test PRIVATE
    TestKalmanFilter.cpp
)
