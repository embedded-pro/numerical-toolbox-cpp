add_executable(dsp.estimators_test)
emil_build_for(dsp.estimators_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.estimators_test)

target_link_libraries(dsp.estimators_test PUBLIC
    gmock_main
    dsp.estimators
)

target_sources(dsp.estimators_test PRIVATE
    TestLinearRegression.cpp
    TestYuleWalker.cpp
)
