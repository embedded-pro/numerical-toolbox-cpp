add_executable(dsp.solvers_test)
emil_build_for(dsp.solvers_test BOOL CONTROL_TOOLBOX_BUILD_TESTS)
emil_add_test(dsp.solvers_test)

target_link_libraries(dsp.solvers_test PUBLIC
    gmock_main
    dsp.solver
)

target_sources(dsp.solvers_test PRIVATE
    TestLevinsonDurbin.cpp
)
