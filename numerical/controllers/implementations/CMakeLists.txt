add_library(numerical.controllers.implementations INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.controllers.implementations INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.implementations INTERFACE
    numerical.controllers.interfaces
)

target_sources(numerical.controllers.implementations PRIVATE
    PidAbsolute.hpp
    PidIncremental.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
