find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.controllers.pid_controller)
emil_build_for(numerical.simulator.controllers.pid_controller HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.controllers.pid_controller PRIVATE
    numerical.simulator.controllers.pid.application
    numerical.simulator.controllers.pid.view
)

target_sources(numerical.simulator.controllers.pid_controller PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.pid_controller PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.pid_controller PRIVATE
        -Wno-error
    )
endif()
