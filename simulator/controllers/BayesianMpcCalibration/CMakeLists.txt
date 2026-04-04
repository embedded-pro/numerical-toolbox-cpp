find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.controllers.bayesian_mpc_calibration)
emil_build_for(numerical.simulator.controllers.bayesian_mpc_calibration HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.controllers.bayesian_mpc_calibration PRIVATE
    numerical.simulator.controllers.bayesian_mpc_calibration.application
    numerical.simulator.controllers.bayesian_mpc_calibration.view
)

target_sources(numerical.simulator.controllers.bayesian_mpc_calibration PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration PRIVATE
        -Wno-error
    )
endif()
