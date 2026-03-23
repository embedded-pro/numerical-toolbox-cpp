if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.math ${EMIL_EXCLUDE_FROM_ALL} STATIC)
    set(_math_visibility PUBLIC)
else()
    add_library(numerical.math ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_math_visibility INTERFACE)
endif()

target_include_directories(numerical.math ${_math_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.math ${_math_visibility}
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
