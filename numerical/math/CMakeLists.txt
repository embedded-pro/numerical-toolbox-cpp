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
    Geometry3D.hpp
    HyperbolicFunctions.hpp
    Matrix.hpp
    QNumber.hpp
    RecursiveBuffer.hpp
    SingleInstructionMultipleData.hpp
    Statistics.hpp
    Toeplitz.hpp
    Tolerance.hpp
    TrigonometricFunctions.hpp
)

numerical_add_coverage_sources(numerical.math
    ComplexNumber.cpp
    Matrix.cpp
    QNumber.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
