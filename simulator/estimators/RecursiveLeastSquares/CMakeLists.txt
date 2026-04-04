find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.estimators.rls)
emil_build_for(numerical.simulator.estimators.rls HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.estimators.rls PRIVATE
    numerical.simulator.estimators.rls.application
    numerical.simulator.estimators.rls.view
)

target_sources(numerical.simulator.estimators.rls PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.estimators.rls PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.estimators.rls PRIVATE
        -Wno-error
    )
endif()
