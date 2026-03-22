add_library(numerical.dynamics ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.dynamics PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.dynamics PUBLIC
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.dynamics PRIVATE
    ArticulatedBodyAlgorithm.cpp
    ArticulatedBodyAlgorithm.hpp
    EulerLagrangeDynamics.hpp
    EulerLagrangeSolver.cpp
    EulerLagrangeSolver.hpp
    ForwardKinematics.cpp
    ForwardKinematics.hpp
    NewtonEulerBody.hpp
    NewtonEulerSolver.cpp
    NewtonEulerSolver.hpp
    RecursiveNewtonEuler.cpp
    RecursiveNewtonEuler.hpp
    RevoluteJointLink.hpp
)

add_subdirectory(test)
