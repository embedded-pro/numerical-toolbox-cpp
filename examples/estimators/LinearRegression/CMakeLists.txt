add_executable(numerical.examples.estimators.linear_regression)
emil_build_for(numerical.examples.estimators.linear_regression HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.estimators.linear_regression PUBLIC
    sciplot
    numerical.estimators
)

target_sources(numerical.examples.estimators.linear_regression PRIVATE
    LinearRegression.cpp
)

if(MSVC)
    target_compile_options(numerical.examples.estimators.linear_regression PRIVATE /W4)
else()
    target_compile_options(numerical.examples.estimators.linear_regression PRIVATE 
        -Wno-error
    )
endif()
