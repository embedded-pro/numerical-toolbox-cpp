add_executable(numerical.estimators.online_test)
emil_build_for(numerical.estimators.online_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.estimators.online_test)

target_link_libraries(numerical.estimators.online_test PUBLIC
    gmock_main
    numerical.estimators.online
)

numerical_add_qemu_test(numerical.estimators.online_test)

target_sources(numerical.estimators.online_test PRIVATE
    TestRecursiveLeastSquares.cpp
    TestLmsAdaptiveFilter.cpp
)
