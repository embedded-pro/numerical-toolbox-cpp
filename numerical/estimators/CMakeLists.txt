add_library(numerical.estimators ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.estimators PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.estimators PRIVATE
    Estimator.hpp
    LinearRegression.cpp
    LinearRegression.hpp
    RecursiveLeastSquares.cpp
    RecursiveLeastSquares.hpp
    YuleWalker.hpp
)

add_subdirectory(test)
