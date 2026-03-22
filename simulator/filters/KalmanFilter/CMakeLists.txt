find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.filters.kalman)
emil_build_for(numerical.simulator.filters.kalman HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.filters.kalman PRIVATE
    numerical.simulator.filters.kalman.application
    numerical.simulator.filters.kalman.view
)

target_sources(numerical.simulator.filters.kalman PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.kalman PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.kalman PRIVATE
        -Wno-error
    )
endif()
