add_executable(dsp.examples.filters.active.kalman_filter)
emil_build_for(dsp.examples.filters.active.kalman_filter PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.filters.active.kalman_filter PUBLIC
    sciplot
    dsp.filters.active
)

target_sources(dsp.examples.filters.active.kalman_filter PRIVATE
    KalmanFilter.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.filters.active.kalman_filter PRIVATE /W4)
else()
    target_compile_options(dsp.examples.filters.active.kalman_filter PRIVATE 
        -Wno-error
    )
endif()
