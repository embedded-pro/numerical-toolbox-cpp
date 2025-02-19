add_executable(dsp.optimizers_test)
emil_build_for(dsp.optimizers_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.optimizers_test)

target_link_libraries(dsp.optimizers_test PUBLIC
    gmock_main
    dsp.estimators
)

target_sources(dsp.optimizers_test PRIVATE
    TestGradientDescent.cpp
)
