add_executable(numerical.simulator.filters.iir.application_test)
emil_build_for(numerical.simulator.filters.iir.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.filters.iir.application_test)

target_link_libraries(numerical.simulator.filters.iir.application_test PUBLIC
    gmock_main
    numerical.simulator.filters.iir.application
)

target_sources(numerical.simulator.filters.iir.application_test PRIVATE
    TestIirForm.cpp
)
