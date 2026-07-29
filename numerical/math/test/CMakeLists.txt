add_executable(numerical.math_test)
emil_build_for(numerical.math_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.math_test)

target_link_libraries(numerical.math_test PUBLIC
    gmock_main
    numerical.math
)

target_sources(numerical.math_test PRIVATE
    TestComplexNumber.cpp
    TestCordic.cpp
    TestLinearTimeInvariant.cpp
    TestMatrixNorms.cpp
    TestQNumber.cpp
    TestMatrix.cpp
    TestQuaternion.cpp
    TestRecursiveBuffer.cpp
    TestStatistics.cpp
    TestToeplitz.cpp
)
