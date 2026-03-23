if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.filters.active ${EMIL_EXCLUDE_FROM_ALL})
    set(_filters_active_visibility PUBLIC)
else()
    add_library(numerical.filters.active ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_filters_active_visibility INTERFACE)
endif()

target_include_directories(numerical.filters.active ${_filters_active_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.active ${_filters_active_visibility}
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

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.filters.active PRIVATE
        ExtendedKalmanFilter.cpp
        KalmanFilter.cpp
        KalmanFilterBase.cpp
        UnscentedKalmanFilter.cpp
    )
endif()

add_subdirectory(test)
