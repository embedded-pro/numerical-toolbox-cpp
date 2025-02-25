add_executable(numerical.neural_network.losses_test)
emil_build_for(numerical.neural_network.losses_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.neural_network.losses_test)

target_link_libraries(numerical.neural_network.losses_test PUBLIC
    gmock_main
    numerical.neural_network.losses
)

target_sources(numerical.neural_network.losses_test PRIVATE
    TestBinaryCrossEntropy.cpp
    TestCategoricalCrossEntropy.cpp
    TestMeanAbsoluteError.cpp
    TestMeanSquaredError.cpp
)
