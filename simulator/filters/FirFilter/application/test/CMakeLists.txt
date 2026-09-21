add_executable(numerical.simulator.filters.fir.application_test)
emil_build_for(numerical.simulator.filters.fir.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.filters.fir.application_test)

target_link_libraries(numerical.simulator.filters.fir.application_test PUBLIC
    gmock_main
    numerical.simulator.filters.fir.application
)

target_sources(numerical.simulator.filters.fir.application_test PRIVATE
    TestFirForm.cpp
)
