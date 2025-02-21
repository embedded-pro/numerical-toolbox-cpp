add_library(dsp.analysis ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.analysis PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.analysis PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.analysis PRIVATE
    DiscreteCosineTransform.hpp
    FastFourierTransform.hpp
    FastFourierTransformRadix2Impl.hpp
    FrequencyResponse.cpp
    FrequencyResponse.hpp
    PowerDensitySpectrum.hpp
)

add_subdirectory(test)
