add_executable(dsp.examples.analysis.filter_frequency_response)
emil_build_for(dsp.examples.analysis.filter_frequency_response HOST Windows PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.analysis.filter_frequency_response PUBLIC
    sciplot
    dsp.analysis
)

target_sources(dsp.examples.analysis.filter_frequency_response PRIVATE
    FilterFrequencyResponse.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.analysis.filter_frequency_response PRIVATE /W4)
else()
    target_compile_options(dsp.examples.analysis.filter_frequency_response PRIVATE 
        -Wno-error
    )
endif()
