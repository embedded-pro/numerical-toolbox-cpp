numerical_add_header_library(numerical.estimators.offline)

target_include_directories(numerical.estimators.offline ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators.offline ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.filters.active
    numerical.math
    numerical.solver
)

target_sources(numerical.estimators.offline PRIVATE
    ExpectationMaximization.hpp
    LinearRegression.hpp
    YuleWalker.hpp
)

numerical_add_coverage_sources(numerical.estimators.offline
    ExpectationMaximization.cpp
    LinearRegression.cpp
    YuleWalker.cpp
)

add_subdirectory(test)
