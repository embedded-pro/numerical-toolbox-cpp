add_executable(dsp.examples.analysis.fast_fourier_transform_radix_2)
emil_build_for(dsp.examples.analysis.fast_fourier_transform_radix_2 PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.analysis.fast_fourier_transform_radix_2 PUBLIC
    sciplot
    dsp.analysis
)

target_sources(dsp.examples.analysis.fast_fourier_transform_radix_2 PRIVATE
    FastFourierTransformRadix2.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.analysis.fast_fourier_transform_radix_2 PRIVATE /W4)
else()
    target_compile_options(dsp.examples.analysis.fast_fourier_transform_radix_2 PRIVATE 
        -Wno-error
    )
endif()
