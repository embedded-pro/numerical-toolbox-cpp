add_executable(numerical.simulator.analysis.psd.application_test)
emil_build_for(numerical.simulator.analysis.psd.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.analysis.psd.application_test)

target_link_libraries(numerical.simulator.analysis.psd.application_test PUBLIC
    gmock_main
    numerical.simulator.analysis.psd.application
)

target_sources(numerical.simulator.analysis.psd.application_test PRIVATE
    TestPsdForm.cpp
)
