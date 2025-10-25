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
    HyperbolicFunctions.hpp
    Matrix.cpp
    Matrix.hpp
    QNumber.cpp
    QNumber.hpp
    RecursiveBuffer.cpp
    RecursiveBuffer.hpp
    SingleInstructionMultipleData.hpp
    Statistics.hpp
    Toeplitz.hpp
    Tolerance.hpp
    TrigonometricFunctions.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
