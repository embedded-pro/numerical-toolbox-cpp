add_library(numerical.simulator.dynamics.robotarm.view STATIC)

target_include_directories(numerical.simulator.dynamics.robotarm.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.dynamics.robotarm.view PUBLIC
    numerical.simulator.dynamics.robotarm.application
    Qt6::Widgets
)

set_target_properties(numerical.simulator.dynamics.robotarm.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.dynamics.robotarm.view PRIVATE
    RobotArm3DWidget.cpp
    RobotArm3DWidget.hpp
    RobotArmConfigurationPanel.cpp
    RobotArmConfigurationPanel.hpp
    RobotArmMainWindow.cpp
    RobotArmMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.dynamics.robotarm.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.dynamics.robotarm.view PRIVATE
        -Wno-error
    )
endif()
