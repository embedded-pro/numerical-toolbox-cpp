add_executable(numerical.simulator.estimators.rls.application_test)
emil_build_for(numerical.simulator.estimators.rls.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.estimators.rls.application_test)

target_link_libraries(numerical.simulator.estimators.rls.application_test PUBLIC
    gmock_main
    numerical.simulator.estimators.rls.application
)

target_sources(numerical.simulator.estimators.rls.application_test PRIVATE
    TestRlsForm.cpp
)
