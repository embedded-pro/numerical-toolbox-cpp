add_executable(numerical.examples.filters.active.kalman_filter)
emil_build_for(numerical.examples.filters.active.kalman_filter HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.filters.active.kalman_filter PUBLIC
    sciplot
    numerical.filters.active
)

target_sources(numerical.examples.filters.active.kalman_filter PRIVATE
    KalmanFilter.cpp
)

target_include_directories(numerical.examples.filters.active.kalman_filter SYSTEM PRIVATE ${sciplot_SOURCE_DIR})

if(MSVC)
    target_compile_options(numerical.examples.filters.active.kalman_filter PRIVATE /W4)
else()
    target_compile_options(numerical.examples.filters.active.kalman_filter PRIVATE 
        -Wno-error
    )
endif()
