add_library(numerical.simulator.filters.iir.application STATIC)

target_include_directories(numerical.simulator.filters.iir.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.iir.application PUBLIC
    numerical.analysis
    numerical.filters.passive
    numerical.simulator.utils
)

target_sources(numerical.simulator.filters.iir.application PRIVATE
    IirFilterSimulator.cpp
    IirFilterSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.iir.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.iir.application PRIVATE
        -Wno-error
    )
endif()
