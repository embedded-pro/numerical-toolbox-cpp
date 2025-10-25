add_executable(numerical.examples.controllers.temperature_controller)
emil_build_for(numerical.examples.controllers.temperature_controller HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.controllers.temperature_controller PUBLIC
    sciplot
    numerical.controllers.implementations
)

target_sources(numerical.examples.controllers.temperature_controller PRIVATE
    TemperatureController.cpp
)

target_include_directories(numerical.examples.controllers.temperature_controller SYSTEM PRIVATE ${sciplot_SOURCE_DIR})

if(MSVC)
    target_compile_options(numerical.examples.controllers.temperature_controller PRIVATE /W4 /wd4100)
else()
    target_compile_options(numerical.examples.controllers.temperature_controller PRIVATE 
        -Wno-error
    )
endif()
