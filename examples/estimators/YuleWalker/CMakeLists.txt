add_executable(numerical.examples.estimators.yule_walker)
emil_build_for(numerical.examples.estimators.yule_walker HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(numerical.examples.estimators.yule_walker PUBLIC
    sciplot
    numerical.estimators
    numerical.solver
)

target_sources(numerical.examples.estimators.yule_walker PRIVATE
    YuleWalker.cpp
)

if(MSVC)
    target_compile_options(numerical.examples.estimators.yule_walker PRIVATE /W4)
else()
    target_compile_options(numerical.examples.estimators.yule_walker PRIVATE 
        -Wno-error
    )
endif()
