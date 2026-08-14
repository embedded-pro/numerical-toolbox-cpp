numerical_add_header_library(numerical.math STATIC)

target_include_directories(numerical.math ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.math ${NUMERICAL_VISIBILITY}
    infra.util
)

target_sources(numerical.math PRIVATE
    CholeskyDecomposition.hpp
    ConstexprMath.hpp
    ComplexNumber.hpp
    ConsistencyMetrics.hpp
    Cordic.hpp
    Geometry3D.hpp
    GivensRotation.hpp
    HouseholderTransform.hpp
    LinearTimeInvariant.hpp
    Matrix.hpp
    MatrixExponential.hpp
    MatrixNorms.hpp
    MatrixOperations.hpp
    QNumber.hpp
    Quaternion.hpp
    RecursiveBuffer.hpp
    Statistics.hpp
    StepResponseMetrics.hpp
    Toeplitz.hpp
    Tolerance.hpp
    Math.hpp
    TriangularSolve.hpp
)

numerical_add_coverage_sources(numerical.math
    Math.cpp
    CholeskyDecomposition.cpp
    ComplexNumber.cpp
    ConsistencyMetrics.cpp
    Cordic.cpp
    LinearTimeInvariant.cpp
    Matrix.cpp
    MatrixExponential.cpp
    QNumber.cpp
    Quaternion.cpp
    RecursiveBuffer.cpp
    Toeplitz.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
add_subdirectory(platform)
