numerical_add_header_library(numerical.estimators)

target_include_directories(numerical.estimators ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.estimators PRIVATE
    ConsistencyMetrics.hpp
    Estimator.hpp
)

numerical_add_coverage_sources(numerical.estimators
    ConsistencyMetrics.cpp
)

add_subdirectory(offline)
add_subdirectory(online)
add_subdirectory(test)
