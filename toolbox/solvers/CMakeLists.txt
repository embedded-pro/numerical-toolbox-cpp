add_library(numerical.solver ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.solver PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.solver PRIVATE
    LevinsonDurbin.hpp
    Solver.cpp
    Solver.hpp
)

add_subdirectory(test)
