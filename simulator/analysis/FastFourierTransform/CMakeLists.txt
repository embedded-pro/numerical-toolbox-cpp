find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.analysis.fast_fourier_transform)
emil_build_for(numerical.simulator.analysis.fast_fourier_transform HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.simulator.analysis.fast_fourier_transform PRIVATE
    numerical.simulator.analysis.fft.application
    numerical.simulator.analysis.fft.view
)

target_sources(numerical.simulator.analysis.fast_fourier_transform PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.fast_fourier_transform PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.fast_fourier_transform PRIVATE
        -Wno-error
    )
endif()
