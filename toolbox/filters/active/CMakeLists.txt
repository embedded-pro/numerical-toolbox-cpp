add_library(numerical.filters.active ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.filters.active INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.active INTERFACE
    infra.util
    numerical.math
)

target_sources(numerical.filters.active PRIVATE
    KalmanFilter.hpp
)

add_subdirectory(test)
