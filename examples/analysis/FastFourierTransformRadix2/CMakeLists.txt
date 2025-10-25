add_executable(numerical.examples.analysis.fast_fourier_transform_radix_2)
emil_build_for(numerical.examples.analysis.fast_fourier_transform_radix_2 HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.analysis.fast_fourier_transform_radix_2 PUBLIC
    sciplot
    numerical.analysis
)

target_sources(numerical.examples.analysis.fast_fourier_transform_radix_2 PRIVATE
    FastFourierTransformRadix2.cpp
)

target_include_directories(numerical.examples.analysis.fast_fourier_transform_radix_2 SYSTEM PRIVATE ${sciplot_SOURCE_DIR})

if(MSVC)
    target_compile_options(numerical.examples.analysis.fast_fourier_transform_radix_2 PRIVATE /W4)
else()
    target_compile_options(numerical.examples.analysis.fast_fourier_transform_radix_2 PRIVATE 
        -Wno-error
    )
endif()
