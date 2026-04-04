numerical_add_header_library(numerical.kinematics)

target_include_directories(numerical.kinematics ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.kinematics ${NUMERICAL_VISIBILITY}
    numerical.dynamics
    numerical.math
    numerical.solver
)

target_sources(numerical.kinematics PRIVATE
    ForwardKinematics.hpp
    InverseKinematics.hpp
)

numerical_add_coverage_sources(numerical.kinematics
    ForwardKinematics.cpp
    InverseKinematics.cpp
)

add_subdirectory(test)
