add_library(numerical.simulator.controllers.lqr.application STATIC)

target_include_directories(numerical.simulator.controllers.lqr.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.lqr.application PUBLIC
    numerical.controllers.implementations
    numerical.solver
)

target_sources(numerical.simulator.controllers.lqr.application PRIVATE
    CartPolePlant.cpp
    CartPolePlant.hpp
    LqrCartPoleSimulator.cpp
    LqrCartPoleSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqr.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqr.application PRIVATE
        -Wno-error
    )
endif()
