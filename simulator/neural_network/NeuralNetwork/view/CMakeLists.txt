add_library(numerical.simulator.neural_network.nn.view STATIC)

set_target_properties(numerical.simulator.neural_network.nn.view PROPERTIES AUTOMOC ON)

target_include_directories(numerical.simulator.neural_network.nn.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
)

target_link_libraries(numerical.simulator.neural_network.nn.view PUBLIC
    numerical.simulator.neural_network.nn.application
    numerical.simulator.widgets
    Qt6::Widgets
)

target_sources(numerical.simulator.neural_network.nn.view PRIVATE
    NnConfigurationPanel.hpp
    NnConfigurationPanel.cpp
    NnMainWindow.hpp
    NnMainWindow.cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(numerical.simulator.neural_network.nn.view PRIVATE -Wno-error)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(numerical.simulator.neural_network.nn.view PRIVATE /W4 /wd4244 /wd4100)
endif()
