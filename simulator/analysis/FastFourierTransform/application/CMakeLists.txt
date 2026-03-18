add_library(numerical.simulator.analysis.fft.application STATIC)

target_include_directories(numerical.simulator.analysis.fft.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.analysis.fft.application PUBLIC
    numerical.analysis
)

target_sources(numerical.simulator.analysis.fft.application PRIVATE
    FftSimulator.cpp
    FftSimulator.hpp
    SignalGenerator.cpp
    SignalGenerator.hpp
    TwiddleFactorsTable.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.analysis.fft.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.analysis.fft.application PRIVATE
        -Wno-error
    )
endif()
