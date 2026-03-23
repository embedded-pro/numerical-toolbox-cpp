if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.dynamics ${EMIL_EXCLUDE_FROM_ALL})
    set(_dynamics_visibility PUBLIC)
else()
    add_library(numerical.dynamics ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_dynamics_visibility INTERFACE)
endif()

target_include_directories(numerical.dynamics ${_dynamics_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.dynamics ${_dynamics_visibility}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.dynamics PRIVATE
    ArticulatedBodyAlgorithm.hpp
    EulerLagrangeDynamics.hpp
    EulerLagrangeSolver.hpp
    NewtonEulerBody.hpp
    NewtonEulerSolver.hpp
    RecursiveNewtonEuler.hpp
    RevoluteJointLink.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.dynamics PRIVATE
        ArticulatedBodyAlgorithm.cpp
        EulerLagrangeSolver.cpp
        NewtonEulerSolver.cpp
        RecursiveNewtonEuler.cpp
    )
endif()

add_subdirectory(test)
