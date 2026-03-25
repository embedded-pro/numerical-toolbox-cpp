numerical_add_header_library(numerical.analysis)

target_include_directories(numerical.analysis ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.analysis ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.analysis.windowing
    numerical.math
)

target_sources(numerical.analysis PRIVATE
    DiscreteCosineTransform.hpp
    FastFourierTransform.hpp
    FastFourierTransformRadix2Impl.hpp
    PowerDensitySpectrum.hpp
)

numerical_add_coverage_sources(numerical.analysis
    FastFourierTransformRadix2Impl.cpp
    PowerDensitySpectrum.cpp
)

add_subdirectory(test)
add_subdirectory(windowing)
