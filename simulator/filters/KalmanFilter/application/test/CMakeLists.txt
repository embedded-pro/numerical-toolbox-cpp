add_executable(numerical.simulator.filters.kalman.application_test)
emil_build_for(numerical.simulator.filters.kalman.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.filters.kalman.application_test)

target_link_libraries(numerical.simulator.filters.kalman.application_test PUBLIC
    gmock_main
    numerical.simulator.filters.kalman.application
)

target_sources(numerical.simulator.filters.kalman.application_test PRIVATE
    TestKalmanForm.cpp
)
