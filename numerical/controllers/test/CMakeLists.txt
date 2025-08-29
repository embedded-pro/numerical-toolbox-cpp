add_executable(numerical.controllers_test)
emil_build_for(numerical.controllers_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.controllers_test)

target_link_libraries(numerical.controllers_test PUBLIC
    gmock_main
    numerical.controllers
    numerical.math_test_helper
)

target_sources(numerical.controllers_test PRIVATE
    TestFieldOrientedController.cpp
    TestPid.cpp
    TestSpaceVectorModulation.cpp
    TestTrajectoryGeneratorTrapezoidal.cpp
    TestTransformsClarkePark.cpp
)
