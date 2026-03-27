add_library(numerical.simulator.filters.iir.view STATIC)

target_include_directories(numerical.simulator.filters.iir.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.iir.view PUBLIC
    numerical.simulator.filters.iir.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.filters.iir.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.filters.iir.view PRIVATE
    IirConfigurationPanel.cpp
    IirConfigurationPanel.hpp
    IirMainWindow.cpp
    IirMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.iir.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.iir.view PRIVATE
        -Wno-error
    )
endif()
