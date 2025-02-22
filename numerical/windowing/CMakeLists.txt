add_library(numerical.windowing ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.windowing INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.windowing INTERFACE
    numerical.math
)

target_sources(numerical.windowing PRIVATE
    Windowing.hpp
)

add_subdirectory(test)
