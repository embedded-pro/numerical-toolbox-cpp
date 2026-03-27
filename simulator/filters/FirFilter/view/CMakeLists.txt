add_library(numerical.simulator.filters.fir.view STATIC)

target_include_directories(numerical.simulator.filters.fir.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.fir.view PUBLIC
    numerical.simulator.filters.fir.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.filters.fir.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.filters.fir.view PRIVATE
    FirConfigurationPanel.cpp
    FirConfigurationPanel.hpp
    FirMainWindow.cpp
    FirMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.fir.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.fir.view PRIVATE
        -Wno-error
    )
endif()
