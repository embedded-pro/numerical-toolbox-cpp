numerical_add_header_library(numerical.nonlinear_control STATIC)

target_include_directories(numerical.nonlinear_control ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.nonlinear_control ${NUMERICAL_VISIBILITY}
    numerical.math
    numerical.solver
    infra.util
)

target_sources(numerical.nonlinear_control PRIVATE
    BacksteppingControl.hpp
    FeedbackLinearization.hpp
    ModelReferenceAdaptiveControl.hpp
)

numerical_add_coverage_sources(numerical.nonlinear_control
    BacksteppingControl.cpp
    FeedbackLinearization.cpp
    ModelReferenceAdaptiveControl.cpp
)

add_subdirectory(test)
