add_library(numerical.simulator.controllers.lqg.view STATIC)

target_include_directories(numerical.simulator.controllers.lqg.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.lqg.view PUBLIC
    numerical.simulator.controllers.lqg.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.controllers.lqg.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.controllers.lqg.view PRIVATE
    LqgConfigurationPanel.cpp
    LqgConfigurationPanel.hpp
    LqgMainWindow.cpp
    LqgMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqg.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqg.view PRIVATE
        -Wno-error
    )
endif()
