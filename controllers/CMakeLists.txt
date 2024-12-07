add_library(toolbox.controllers ${EMIL_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(toolbox.controllers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(toolbox.controllers PUBLIC
    infra.util
)

target_sources(toolbox.controllers PRIVATE
    Pid.hpp
    Pid.cpp
)

add_subdirectory(test)
