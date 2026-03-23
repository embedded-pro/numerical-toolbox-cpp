add_library(numerical.windowing ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.windowing PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.windowing PUBLIC
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
