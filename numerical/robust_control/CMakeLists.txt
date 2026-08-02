numerical_add_header_library(numerical.robust_control STATIC)

target_include_directories(numerical.robust_control ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.robust_control ${NUMERICAL_VISIBILITY}
    numerical.math
    numerical.filters.passive
    numerical.solver
    numerical.controllers.implementations
    infra.util
)

target_sources(numerical.robust_control PRIVATE
    ActiveDisturbanceRejection.hpp
    DisturbanceObserver.hpp
    HInfinityStateFeedback.hpp
    SlidingModeControl.hpp
)

numerical_add_coverage_sources(numerical.robust_control
    ActiveDisturbanceRejection.cpp
    DisturbanceObserver.cpp
    HInfinityStateFeedback.cpp
    SlidingModeControl.cpp
)

add_subdirectory(test)
