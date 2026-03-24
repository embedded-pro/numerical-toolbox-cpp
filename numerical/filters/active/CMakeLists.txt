numerical_add_header_library(numerical.filters.active)

target_include_directories(numerical.filters.active ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.active ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.filters.active PRIVATE
    ExtendedKalmanFilter.hpp
    KalmanFilter.hpp
    KalmanFilterBase.hpp
    UnscentedKalmanFilter.hpp
)

numerical_add_coverage_sources(numerical.filters.active
    ExtendedKalmanFilter.cpp
    KalmanFilter.cpp
    KalmanFilterBase.cpp
    UnscentedKalmanFilter.cpp
)

add_subdirectory(test)
