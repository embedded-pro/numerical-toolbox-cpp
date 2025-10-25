add_executable(numerical.examples.neural_network.squared_x)
emil_build_for(numerical.examples.neural_network.squared_x HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.neural_network.squared_x PUBLIC
    sciplot
    #numerical.neural_network
)

target_sources(numerical.examples.neural_network.squared_x PRIVATE
    Main.cpp
)

target_include_directories(numerical.examples.neural_network.squared_x SYSTEM PRIVATE ${sciplot_SOURCE_DIR})

if(MSVC)
    target_compile_options(numerical.examples.neural_network.squared_x PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.examples.neural_network.squared_x PRIVATE 
        -Wno-error
    )
endif()
