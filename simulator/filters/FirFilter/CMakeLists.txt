find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.filters.fir_filter)
emil_build_for(numerical.simulator.filters.fir_filter HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.filters.fir_filter PRIVATE
    numerical.simulator.filters.fir.application
    numerical.simulator.filters.fir.view
)

target_sources(numerical.simulator.filters.fir_filter PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.fir_filter PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.fir_filter PRIVATE
        -Wno-error
    )
endif()
