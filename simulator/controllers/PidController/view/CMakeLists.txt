add_library(numerical.simulator.controllers.pid.view STATIC)

target_include_directories(numerical.simulator.controllers.pid.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.pid.view PUBLIC
    numerical.simulator.controllers.pid.application
    Qt6::Widgets
)

set_target_properties(numerical.simulator.controllers.pid.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.controllers.pid.view PRIVATE
    PidChartWidget.cpp
    PidChartWidget.hpp
    PidConfigurationPanel.cpp
    PidConfigurationPanel.hpp
    PidMainWindow.cpp
    PidMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.pid.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.pid.view PRIVATE
        -Wno-error
    )
endif()
