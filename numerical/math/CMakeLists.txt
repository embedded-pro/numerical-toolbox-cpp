numerical_add_header_library(numerical.math STATIC)

target_include_directories(numerical.math ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.math ${NUMERICAL_VISIBILITY}
    infra.util
)

target_sources(numerical.math PRIVATE
    AdvancedFunctions.hpp
    ComplexNumber.hpp
    Cordic.hpp
    Geometry3D.hpp
    GivensRotation.hpp
    HouseholderTransform.hpp
    HyperbolicFunctions.hpp
    LinearTimeInvariant.hpp
    Matrix.hpp
    MatrixNorms.hpp
    QNumber.hpp
    Quaternion.hpp
    RecursiveBuffer.hpp
    SingleInstructionMultipleData.hpp
    Statistics.hpp
    StepResponseMetrics.hpp
    Toeplitz.hpp
    Tolerance.hpp
    TriangularSolve.hpp
    TrigonometricFunctions.hpp
)

numerical_add_coverage_sources(numerical.math
    ComplexNumber.cpp
    Cordic.cpp
    LinearTimeInvariant.cpp
    Matrix.cpp
    QNumber.cpp
    Quaternion.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
