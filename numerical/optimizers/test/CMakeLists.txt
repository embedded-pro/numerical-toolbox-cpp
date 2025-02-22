add_executable(numerical.optimizers_test)
emil_build_for(numerical.optimizers_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.optimizers_test)

target_link_libraries(numerical.optimizers_test PUBLIC
    gmock_main
    numerical.estimators
)

target_sources(numerical.optimizers_test PRIVATE
    TestGradientDescent.cpp
)
