add_library(numerical.simulator.analysis.psd.application STATIC)

target_include_directories(numerical.simulator.analysis.psd.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.analysis.psd.application PUBLIC
    numerical.analysis
    numerical.windowing
    numerical.simulator.utils
)

target_sources(numerical.simulator.analysis.psd.application PRIVATE
    PsdSimulator.cpp
    PsdSimulator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.psd.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.psd.application PRIVATE
        -Wno-error
    )
endif()
