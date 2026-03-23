add_library(numerical.simulator.filters.kalman.view STATIC)

target_include_directories(numerical.simulator.filters.kalman.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.kalman.view PUBLIC
    numerical.simulator.filters.kalman.application
    Qt6::Widgets
)

set_target_properties(numerical.simulator.filters.kalman.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.filters.kalman.view PRIVATE
    KalmanChartWidget.cpp
    KalmanChartWidget.hpp
    KalmanConfigPanel.cpp
    KalmanConfigPanel.hpp
    KalmanMainWindow.cpp
    KalmanMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.kalman.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.kalman.view PRIVATE
        -Wno-error
    )
endif()
