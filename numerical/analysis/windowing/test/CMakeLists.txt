add_executable(numerical.analysis.windowing_test)
emil_build_for(numerical.analysis.windowing_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.analysis.windowing_test)

target_link_libraries(numerical.analysis.windowing_test PUBLIC
    gmock_main
    numerical.analysis.windowing
)

target_sources(numerical.analysis.windowing_test PRIVATE
    TestWindowing.cpp
)
