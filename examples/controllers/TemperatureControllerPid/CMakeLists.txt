add_executable(dsp.examples.controllers.temperature_controller)
emil_build_for(dsp.examples.controllers.temperature_controller HOST Windows PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.controllers.temperature_controller PUBLIC
    sciplot
    dsp.controllers
)

target_sources(dsp.examples.controllers.temperature_controller PRIVATE
    TemperatureController.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.controllers.temperature_controller PRIVATE /W4)
else()
    target_compile_options(dsp.examples.controllers.temperature_controller PRIVATE 
        -Wno-error
    )
endif()
