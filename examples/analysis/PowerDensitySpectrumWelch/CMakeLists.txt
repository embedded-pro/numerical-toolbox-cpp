add_executable(numerical.examples.analysis.power_density_spectrum_welch)
emil_build_for(numerical.examples.analysis.power_density_spectrum_welch HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.analysis.power_density_spectrum_welch PUBLIC
    sciplot
    numerical.analysis
)

target_sources(numerical.examples.analysis.power_density_spectrum_welch PRIVATE
    PowerDensitySpectrumWelch.cpp
)

target_include_directories(numerical.examples.analysis.power_density_spectrum_welch SYSTEM PRIVATE ${sciplot_SOURCE_DIR})

if(MSVC)
    target_compile_options(numerical.examples.analysis.power_density_spectrum_welch PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.examples.analysis.power_density_spectrum_welch PRIVATE 
        -Wno-error
    )
endif()
