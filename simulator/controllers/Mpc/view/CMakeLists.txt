add_library(numerical.simulator.controllers.mpc.view STATIC)

target_include_directories(numerical.simulator.controllers.mpc.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.mpc.view PUBLIC
    numerical.simulator.controllers.mpc.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.controllers.mpc.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.controllers.mpc.view PRIVATE
    MpcConfigurationPanel.cpp
    MpcConfigurationPanel.hpp
    MpcMainWindow.cpp
    MpcMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.mpc.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.mpc.view PRIVATE
        -Wno-error
    )
endif()
