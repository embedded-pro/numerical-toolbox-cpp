find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(numerical.simulator.filters.iir_filter)
emil_build_for(numerical.simulator.filters.iir_filter HOST All PREREQUISITE_BOOL NUMERICAL_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(numerical.simulator.filters.iir_filter PRIVATE
    numerical.simulator.filters.iir.application
    numerical.simulator.filters.iir.view
)

target_sources(numerical.simulator.filters.iir_filter PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.iir_filter PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.iir_filter PRIVATE
        -Wno-error
    )
endif()
