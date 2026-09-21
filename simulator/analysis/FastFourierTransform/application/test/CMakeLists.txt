add_executable(numerical.simulator.analysis.fft.application_test)
emil_build_for(numerical.simulator.analysis.fft.application_test HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(numerical.simulator.analysis.fft.application_test)

target_link_libraries(numerical.simulator.analysis.fft.application_test PUBLIC
    gmock_main
    numerical.simulator.analysis.fft.application
)

target_sources(numerical.simulator.analysis.fft.application_test PRIVATE
    TestFftForm.cpp
)
