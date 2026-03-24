add_library(numerical.simulator.utils STATIC)

target_include_directories(numerical.simulator.utils PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.utils PUBLIC
    numerical.analysis
)

target_sources(numerical.simulator.utils PRIVATE
    SignalGenerator.cpp
    SignalGenerator.hpp
    TwiddleFactorsTable.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.utils PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.utils PRIVATE
        -Wno-error
    )
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_library(numerical.simulator.widgets STATIC)

target_include_directories(numerical.simulator.widgets PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.widgets PUBLIC
    Qt6::Widgets
)

set_target_properties(numerical.simulator.widgets PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.widgets PRIVATE
    TimeSeriesChartWidget.cpp
    TimeSeriesChartWidget.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.widgets PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.widgets PRIVATE
        -Wno-error
    )
endif()
