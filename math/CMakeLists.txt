add_library(dsp.math ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.math PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.math PUBLIC
    infra.util
)

target_sources(dsp.math PRIVATE
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
    TrigonometricFunctions.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
