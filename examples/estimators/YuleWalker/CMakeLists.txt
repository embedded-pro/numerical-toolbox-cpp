add_executable(dsp.examples.estimators.yule_walker)
emil_build_for(dsp.examples.estimators.yule_walker PREREQUISITE_BOOL CONTROL_TOOLBOX_BUILD_EXAMPLES)

target_link_libraries(dsp.examples.estimators.yule_walker PUBLIC
    sciplot
    dsp.estimators
    dsp.solver
)

target_sources(dsp.examples.estimators.yule_walker PRIVATE
    YuleWalker.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.estimators.yule_walker PRIVATE /W4)
else()
    target_compile_options(dsp.examples.estimators.yule_walker PRIVATE 
        -Wno-error
    )
endif()
