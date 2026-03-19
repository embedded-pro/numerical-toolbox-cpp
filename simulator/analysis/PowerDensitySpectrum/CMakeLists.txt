find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.analysis.power_density_spectrum)
emil_build_for(numerical.simulator.analysis.power_density_spectrum HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.analysis.power_density_spectrum PRIVATE
    numerical.simulator.analysis.psd.application
    numerical.simulator.analysis.psd.view
)

target_sources(numerical.simulator.analysis.power_density_spectrum PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.power_density_spectrum PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.power_density_spectrum PRIVATE
        -Wno-error
    )
endif()
