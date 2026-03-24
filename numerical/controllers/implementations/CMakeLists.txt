numerical_add_header_library(numerical.controllers.implementations)

target_include_directories(numerical.controllers.implementations ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.implementations ${NUMERICAL_VISIBILITY}
    numerical.controllers.interfaces
    numerical.solver
)

target_sources(numerical.controllers.implementations PRIVATE
    Lqr.hpp
    Mpc.hpp
    PidIncremental.hpp
)

numerical_add_coverage_sources(numerical.controllers.implementations
    Lqr.cpp
    Mpc.cpp
    PidIncremental.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
