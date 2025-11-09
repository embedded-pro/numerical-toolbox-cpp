add_executable(numerical.neural_network.activation_test)
emil_build_for(numerical.neural_network.activation_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.neural_network.activation_test)

target_link_libraries(numerical.neural_network.activation_test PUBLIC
    gmock_main
    numerical.neural_network.activation
)

target_sources(numerical.neural_network.activation_test PRIVATE
    TestLeakyReLU.cpp
    TestReLU.cpp
    TestSigmoid.cpp
    TestSoftmax.cpp
    TestTanh.cpp
)
