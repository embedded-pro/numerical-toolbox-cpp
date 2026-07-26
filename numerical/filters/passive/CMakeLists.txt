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
    CicFilter.hpp
    ExponentialMovingAverage.hpp
    Fir.hpp
    Iir.hpp
    MedianFilter.hpp
    MovingAverage.hpp
)

numerical_add_coverage_sources(numerical.filters.passive
    CicFilter.cpp
    ExponentialMovingAverage.cpp
    Fir.cpp
    Iir.cpp
    MedianFilter.cpp
    MovingAverage.cpp
)

add_subdirectory(test)
