numerical_add_header_library(numerical.estimators.online)

target_include_directories(numerical.estimators.online ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.estimators.online ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.estimators.online PRIVATE
    RecursiveLeastSquares.hpp
)

numerical_add_coverage_sources(numerical.estimators.online
    RecursiveLeastSquares.cpp
)

add_subdirectory(test)
