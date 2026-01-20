add_executable(numerical.estimators_test)
emil_build_for(numerical.estimators_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.estimators_test)

target_link_libraries(numerical.estimators_test PUBLIC
    gmock_main
    numerical.estimators
)

target_sources(numerical.estimators_test PRIVATE
    TestLinearRegression.cpp
    TestRecursiveLeastSquares.cpp
)
