add_library(numerical.simulator.analysis.psd.view STATIC)

target_include_directories(numerical.simulator.analysis.psd.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.analysis.psd.view PUBLIC
    numerical.simulator.analysis.psd.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.analysis.psd.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.analysis.psd.view PRIVATE
    PsdConfigurationPanel.cpp
    PsdConfigurationPanel.hpp
    PsdMainWindow.cpp
    PsdMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.psd.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.psd.view PRIVATE
        -Wno-error
    )
endif()
