add_executable(dsp.examples.analysis.power_density_spectrum_bartlett)
emil_build_for(dsp.examples.analysis.power_density_spectrum_bartlett PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.analysis.power_density_spectrum_bartlett PUBLIC
    sciplot
    dsp.analysis
)

target_sources(dsp.examples.analysis.power_density_spectrum_bartlett PRIVATE
    PowerDensitySpectrumBartlett.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.analysis.power_density_spectrum_bartlett PRIVATE /W4)
else()
    target_compile_options(dsp.examples.analysis.power_density_spectrum_bartlett PRIVATE 
        -Wno-error
    )
endif()
