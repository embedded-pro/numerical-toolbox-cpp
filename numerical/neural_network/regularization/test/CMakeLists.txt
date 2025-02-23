add_executable(numerical.neural_network.regularization_test)
emil_build_for(numerical.neural_network.regularization_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.neural_network.regularization_test)

target_link_libraries(numerical.neural_network.regularization_test PUBLIC
    gmock_main
    numerical.neural_network.regularization
)

target_sources(numerical.neural_network.regularization_test PRIVATE
    TestL1.cpp
    TestL2.cpp
)
