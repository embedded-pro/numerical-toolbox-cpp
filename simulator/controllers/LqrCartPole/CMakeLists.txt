find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.controllers.lqr_cart_pole)
emil_build_for(numerical.simulator.controllers.lqr_cart_pole HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.controllers.lqr_cart_pole PRIVATE
    numerical.simulator.controllers.lqr.application
    numerical.simulator.controllers.lqr.view
)

target_sources(numerical.simulator.controllers.lqr_cart_pole PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqr_cart_pole PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqr_cart_pole PRIVATE
        -Wno-error
    )
endif()
