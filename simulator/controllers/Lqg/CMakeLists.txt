find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.controllers.lqg)
emil_build_for(numerical.simulator.controllers.lqg HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.controllers.lqg PRIVATE
    numerical.simulator.controllers.lqg.application
    numerical.simulator.controllers.lqg.view
)

target_sources(numerical.simulator.controllers.lqg PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqg PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqg PRIVATE
        -Wno-error
    )
endif()
