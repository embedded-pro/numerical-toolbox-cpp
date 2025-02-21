add_executable(numerical.windowing_test)
emil_build_for(numerical.windowing_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.windowing_test)

target_link_libraries(numerical.windowing_test PUBLIC
    gmock_main
    numerical.windowing
)

target_sources(numerical.windowing_test PRIVATE
    TestWindowing.cpp
)
