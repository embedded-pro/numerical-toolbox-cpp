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
