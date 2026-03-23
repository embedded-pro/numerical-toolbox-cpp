if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.estimators ${EMIL_EXCLUDE_FROM_ALL})
    set(_estimators_visibility PUBLIC)
else()
    add_library(numerical.estimators ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_estimators_visibility INTERFACE)
endif()

target_include_directories(numerical.estimators ${_estimators_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators ${_estimators_visibility}
    infra.util
    numerical.math
)

target_sources(numerical.estimators PRIVATE
    Estimator.hpp
    LinearRegression.hpp
    RecursiveLeastSquares.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.estimators PRIVATE
        LinearRegression.cpp
        RecursiveLeastSquares.cpp
    )
endif()

add_subdirectory(test)
