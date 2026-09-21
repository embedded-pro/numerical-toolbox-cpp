add_executable(numerical.simulator.controllers.mpc.application_test)
emil_build_for(numerical.simulator.controllers.mpc.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.controllers.mpc.application_test)

target_link_libraries(numerical.simulator.controllers.mpc.application_test PUBLIC
    gmock_main
    numerical.simulator.controllers.mpc.application
)

target_sources(numerical.simulator.controllers.mpc.application_test PRIVATE
    TestMpcForm.cpp
)
