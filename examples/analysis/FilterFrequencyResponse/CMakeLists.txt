add_executable(numerical.examples.analysis.filter_frequency_response)
emil_build_for(numerical.examples.analysis.filter_frequency_response HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.analysis.filter_frequency_response PUBLIC
    sciplot
    numerical.analysis
)

target_sources(numerical.examples.analysis.filter_frequency_response PRIVATE
    FilterFrequencyResponse.cpp
)

if(MSVC)
    target_compile_options(numerical.examples.analysis.filter_frequency_response PRIVATE /W4)
else()
    target_compile_options(numerical.examples.analysis.filter_frequency_response PRIVATE 
        -Wno-error
    )
endif()
