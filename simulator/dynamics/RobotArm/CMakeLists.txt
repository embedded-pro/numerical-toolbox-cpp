find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.dynamics.robot_arm)
emil_build_for(numerical.simulator.dynamics.robot_arm HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.dynamics.robot_arm PRIVATE
    numerical.simulator.dynamics.robotarm.application
    numerical.simulator.dynamics.robotarm.view
)

target_sources(numerical.simulator.dynamics.robot_arm PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.dynamics.robot_arm PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.dynamics.robot_arm PRIVATE
        -Wno-error
    )
endif()
