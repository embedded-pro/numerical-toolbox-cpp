find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.controllers.mpc)
emil_build_for(numerical.simulator.controllers.mpc HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.controllers.mpc PRIVATE
    numerical.simulator.controllers.mpc.application
    numerical.simulator.controllers.mpc.view
)

target_sources(numerical.simulator.controllers.mpc PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.mpc PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.mpc PRIVATE
        -Wno-error
    )
endif()
