add_library(numerical.math ${EMIL_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(numerical.math PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.math PUBLIC
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

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.math PRIVATE
        ComplexNumber.cpp
        Matrix.cpp
        QNumber.cpp
    )
endif()

add_subdirectory(test)
add_subdirectory(test_doubles)
