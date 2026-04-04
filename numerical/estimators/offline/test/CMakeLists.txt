add_executable(numerical.estimators.offline_test)
emil_build_for(numerical.estimators.offline_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.estimators.offline_test)

target_link_libraries(numerical.estimators.offline_test PUBLIC
    gmock_main
    numerical.estimators.offline
)

target_sources(numerical.estimators.offline_test PRIVATE
    TestLinearRegression.cpp
    TestYuleWalker.cpp
)
