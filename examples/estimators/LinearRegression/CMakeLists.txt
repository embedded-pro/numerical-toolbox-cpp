add_executable(dsp.examples.estimators.linear_regression)
emil_build_for(dsp.examples.estimators.linear_regression PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.estimators.linear_regression PUBLIC
    sciplot
    dsp.estimators
)

target_sources(dsp.examples.estimators.linear_regression PRIVATE
    LinearRegression.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.estimators.linear_regression PRIVATE /W4)
else()
    target_compile_options(dsp.examples.estimators.linear_regression PRIVATE 
        -Wno-error
    )
endif()
