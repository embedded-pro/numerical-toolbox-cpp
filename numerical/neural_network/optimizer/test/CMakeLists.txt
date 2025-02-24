add_executable(numerical.neural_network.optimizer_test)
emil_build_for(numerical.neural_network.optimizer_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.neural_network.optimizer_test)

target_link_libraries(numerical.neural_network.optimizer_test PUBLIC
    gmock_main
    numerical.neural_network.optimizer
)

target_sources(numerical.neural_network.optimizer_test PRIVATE
    TestGradientDescent.cpp
)
