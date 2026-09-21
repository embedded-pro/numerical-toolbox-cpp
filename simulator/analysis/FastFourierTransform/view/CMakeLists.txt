add_library(numerical.simulator.analysis.fft.view STATIC)

target_include_directories(numerical.simulator.analysis.fft.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.analysis.fft.view PUBLIC
    numerical.simulator.analysis.fft.application
    numerical.simulator.shell
    ui.charts
    ui.backend.qt
    Qt6::Widgets
)

set_target_properties(numerical.simulator.analysis.fft.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.analysis.fft.view PRIVATE
    FftMainWindow.cpp
    FftMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.fft.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.fft.view PRIVATE
        -Wno-error
    )
endif()
