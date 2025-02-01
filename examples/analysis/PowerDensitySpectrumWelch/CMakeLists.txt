add_executable(dsp.examples.analysis.power_density_spectrum_welch)
emil_build_for(dsp.examples.analysis.power_density_spectrum_welch PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.analysis.power_density_spectrum_welch PUBLIC
    sciplot
    dsp.analysis
)

target_sources(dsp.examples.analysis.power_density_spectrum_welch PRIVATE
    PowerDensitySpectrumWelch.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.analysis.power_density_spectrum_welch PRIVATE /W4)
else()
    target_compile_options(dsp.examples.analysis.power_density_spectrum_welch PRIVATE 
        -Wno-error
    )
endif()
