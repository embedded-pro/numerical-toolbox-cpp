add_library(numerical.simulator.filters.fir.application STATIC)

target_include_directories(numerical.simulator.filters.fir.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.fir.application PUBLIC
    numerical.analysis
    numerical.filters.passive
    numerical.simulator.utils
)

target_sources(numerical.simulator.filters.fir.application PRIVATE
    FirFilterSimulator.cpp
    FirFilterSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.fir.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.fir.application PRIVATE
        -Wno-error
    )
endif()
