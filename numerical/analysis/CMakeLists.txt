add_library(numerical.analysis ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.analysis PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.analysis PUBLIC
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.analysis PRIVATE
    DiscreteCosineTransform.hpp
    FastFourierTransform.hpp
    FastFourierTransformRadix2Impl.hpp
    FrequencyResponse.hpp
    PowerDensitySpectrum.hpp
    RootLocus.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.analysis PRIVATE
        FastFourierTransformRadix2Impl.cpp
        RootLocus.cpp
    )
endif()

add_subdirectory(test)
