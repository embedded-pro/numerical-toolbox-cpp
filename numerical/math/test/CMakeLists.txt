add_executable(numerical.math_test)
emil_build_for(numerical.math_test BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)
emil_add_test(numerical.math_test)

target_link_libraries(numerical.math_test PUBLIC
    gmock_main
    numerical.math
    numerical.math_test_helper
)

numerical_add_qemu_test(numerical.math_test)

target_sources(numerical.math_test PRIVATE
    TestCholeskyDecomposition.cpp
    TestComplexNumber.cpp
    TestMath.cpp
    TestConsistencyMetrics.cpp
    TestCordic.cpp
    TestGivensRotation.cpp
    TestHouseholderTransform.cpp
    TestLinearTimeInvariant.cpp
    TestMatrixExponential.cpp
    TestMatrixNorms.cpp
    TestMatrixOperations.cpp
    TestQNumber.cpp
    TestTolerance.cpp
    TestMatrix.cpp
    TestQuaternion.cpp
    TestRecursiveBuffer.cpp
    TestStatistics.cpp
    TestStepResponseMetrics.cpp
    TestToeplitz.cpp
    TestTriangularSolve.cpp
)
