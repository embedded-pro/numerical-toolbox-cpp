add_library(numerical.simulator.controllers.lqg.application STATIC)

target_include_directories(numerical.simulator.controllers.lqg.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.lqg.application PUBLIC
    numerical.controllers.implementations
    numerical.filters.active
)

target_sources(numerical.simulator.controllers.lqg.application PRIVATE
    LqgSimulator.cpp
    LqgSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqg.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqg.application PRIVATE
        -Wno-error
    )
endif()
