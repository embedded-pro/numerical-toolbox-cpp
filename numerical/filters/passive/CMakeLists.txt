numerical_add_header_library(numerical.filters.passive)

target_include_directories(numerical.filters.passive ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.passive ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.filters.passive PRIVATE
    BiquadCascade.hpp
    CicFilter.hpp
    ExponentialMovingAverage.hpp
    Fir.hpp
    Iir.hpp
    IirFilterDesign.hpp
    MedianFilter.hpp
    MovingAverage.hpp
    NotchCombFilter.hpp
    SavitzkyGolayFilter.hpp
)

numerical_add_coverage_sources(numerical.filters.passive
    BiquadCascade.cpp
    CicFilter.cpp
    ExponentialMovingAverage.cpp
    Fir.cpp
    Iir.cpp
    IirFilterDesign.cpp
    MedianFilter.cpp
    MovingAverage.cpp
    NotchCombFilter.cpp
    SavitzkyGolayFilter.cpp
)

add_subdirectory(test)
