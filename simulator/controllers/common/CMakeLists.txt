add_library(numerical.simulator.controllers.common STATIC)

target_include_directories(numerical.simulator.controllers.common PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_sources(numerical.simulator.controllers.common PRIVATE
    Plant.cpp
    Plant.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.common PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.common PRIVATE
        -Wno-error
    )
endif()
