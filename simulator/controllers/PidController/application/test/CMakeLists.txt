add_executable(numerical.simulator.controllers.pid.application_test)
emil_build_for(numerical.simulator.controllers.pid.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.controllers.pid.application_test)

target_link_libraries(numerical.simulator.controllers.pid.application_test PUBLIC
    gmock_main
    numerical.simulator.controllers.pid.application
)

target_sources(numerical.simulator.controllers.pid.application_test PRIVATE
    TestPidForm.cpp
)
