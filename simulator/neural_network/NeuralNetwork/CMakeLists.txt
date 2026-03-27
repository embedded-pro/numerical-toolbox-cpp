find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.neural_network.nn)
emil_build_for(numerical.simulator.neural_network.nn HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.neural_network.nn PRIVATE
    numerical.simulator.neural_network.nn.application
    numerical.simulator.neural_network.nn.view
)

target_sources(numerical.simulator.neural_network.nn PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.neural_network.nn PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.neural_network.nn PRIVATE
        -Wno-error
    )
endif()
