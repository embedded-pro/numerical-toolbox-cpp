add_executable(numerical.solvers_test)
emil_build_for(numerical.solvers_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.solvers_test)

target_link_libraries(numerical.solvers_test PUBLIC
    gmock_main
    numerical.solver
)

target_sources(numerical.solvers_test PRIVATE
    TestLevinsonDurbin.cpp
)
