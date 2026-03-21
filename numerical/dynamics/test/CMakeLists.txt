add_executable(numerical.dynamics_test)
emil_build_for(numerical.dynamics_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.dynamics_test)

target_link_libraries(numerical.dynamics_test PUBLIC
    gmock_main
    numerical.dynamics
)

target_sources(numerical.dynamics_test PRIVATE
    TestEulerLagrangeSolver.cpp
)
