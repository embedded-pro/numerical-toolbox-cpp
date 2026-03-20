add_library(numerical.simulator.controllers.pid.application STATIC)

target_include_directories(numerical.simulator.controllers.pid.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.pid.application PUBLIC
    numerical.analysis
    numerical.controllers.implementations
    numerical.simulator.controllers.common
    numerical.solver
)

target_sources(numerical.simulator.controllers.pid.application PRIVATE
    PidSimulator.cpp
    PidSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.pid.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.pid.application PRIVATE
        -Wno-error
    )
endif()
