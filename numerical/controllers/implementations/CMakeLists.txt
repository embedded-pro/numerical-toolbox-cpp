add_library(numerical.controllers.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.controllers.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.implementations PUBLIC
    numerical.controllers.interfaces
)

target_sources(numerical.controllers.implementations PRIVATE
    PidIncremental.cpp
    PidIncremental.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
