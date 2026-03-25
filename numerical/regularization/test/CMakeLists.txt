add_executable(numerical.regularization_test)
emil_build_for(numerical.regularization_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.regularization_test)

target_link_libraries(numerical.regularization_test PUBLIC
    gmock_main
    numerical.regularization
)

target_sources(numerical.regularization_test PRIVATE
    TestL1.cpp
    TestL2.cpp
)
