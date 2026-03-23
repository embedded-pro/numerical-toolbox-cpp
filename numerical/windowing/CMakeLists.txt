if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.windowing ${EMIL_EXCLUDE_FROM_ALL})
    set(_windowing_visibility PUBLIC)
else()
    add_library(numerical.windowing ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_windowing_visibility INTERFACE)
endif()

target_include_directories(numerical.windowing ${_windowing_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.windowing ${_windowing_visibility}
    numerical.math
)

target_sources(numerical.windowing PRIVATE
    Windowing.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.windowing PRIVATE
        Windowing.cpp
    )
endif()

add_subdirectory(test)
