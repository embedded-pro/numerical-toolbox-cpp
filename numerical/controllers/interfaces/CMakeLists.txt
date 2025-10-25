add_library(numerical.controllers.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.controllers.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.interfaces INTERFACE
    infra.util
    numerical.math
)

target_sources(numerical.controllers.interfaces PRIVATE
    PidController.hpp
    PidDriver.hpp
)
