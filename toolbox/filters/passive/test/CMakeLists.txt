add_executable(numerical.filters.passive_test)
emil_build_for(numerical.filters.passive_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.filters.passive_test)

target_link_libraries(numerical.filters.passive_test PUBLIC
    gmock_main
    numerical.filters.passive
    numerical.math_test_helper
    numerical.controllers_test_helper
)

target_sources(numerical.filters.passive_test PRIVATE
    TestFir.cpp
    TestIir.cpp
)
