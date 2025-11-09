add_executable(numerical.neural_network.layer_test)
emil_build_for(numerical.neural_network.layer_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.neural_network.layer_test)

target_link_libraries(numerical.neural_network.layer_test PUBLIC
    gmock_main
    numerical.neural_network.layer
)

target_sources(numerical.neural_network.layer_test PRIVATE
    TestDense.cpp
)
