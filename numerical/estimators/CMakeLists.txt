numerical_add_header_library(numerical.estimators)

target_include_directories(numerical.estimators ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.estimators PRIVATE
    Estimator.hpp
    LinearRegression.hpp
    RecursiveLeastSquares.hpp
)

numerical_add_coverage_sources(numerical.estimators
    LinearRegression.cpp
    RecursiveLeastSquares.cpp
)

add_subdirectory(test)
