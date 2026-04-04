add_executable(numerical.optimization_test)
emil_build_for(numerical.optimization_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.optimization_test)

target_link_libraries(numerical.optimization_test PUBLIC
    gmock_main
    numerical.optimization
)

target_sources(numerical.optimization_test PRIVATE
    TestBayesianOptimization.cpp
    TestGradientDescent.cpp
)
