add_library(dsp.solver ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.solver PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.solver PRIVATE
    LevinsonDurbin.hpp
    Solver.cpp
    Solver.hpp
)

add_subdirectory(test)
