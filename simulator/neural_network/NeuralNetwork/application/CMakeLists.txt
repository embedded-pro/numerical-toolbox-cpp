add_library(numerical.simulator.neural_network.nn.application STATIC)

target_include_directories(numerical.simulator.neural_network.nn.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
)

target_link_libraries(numerical.simulator.neural_network.nn.application PUBLIC
    numerical.neural_network.model
    numerical.regularization
)

target_sources(numerical.simulator.neural_network.nn.application PRIVATE
    NnSimulator.hpp
    NnSimulator.cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(numerical.simulator.neural_network.nn.application PRIVATE -Wno-error)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(numerical.simulator.neural_network.nn.application PRIVATE /W4 /wd4244 /wd4100)
endif()
