if (EMIL_ENABLE_COVERAGE)
    add_library(numerical.controllers.implementations ${EMIL_EXCLUDE_FROM_ALL})
    set(_controllers_impl_visibility PUBLIC)
else()
    add_library(numerical.controllers.implementations ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)
    set(_controllers_impl_visibility INTERFACE)
endif()

target_include_directories(numerical.controllers.implementations ${_controllers_impl_visibility}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.implementations ${_controllers_impl_visibility}
    numerical.controllers.interfaces
    numerical.solver
)

target_sources(numerical.controllers.implementations PRIVATE
    Lqr.hpp
    Mpc.hpp
    PidIncremental.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.controllers.implementations PRIVATE
        Lqr.cpp
        Mpc.cpp
        PidIncremental.cpp
    )
endif()

add_subdirectory(test)
add_subdirectory(test_doubles)
