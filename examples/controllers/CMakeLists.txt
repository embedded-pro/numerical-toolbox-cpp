add_executable(dsp.examples.controllers)

target_link_libraries(dsp.examples.controllers PUBLIC
    sciplot
    dsp.controllers
)

target_sources(dsp.examples.controllers PRIVATE
    Pid.cpp
)

if(MSVC)
    target_compile_options(dsp.examples.controllers PRIVATE /W4)
else()
    target_compile_options(dsp.examples.controllers PRIVATE 
        -Wno-error
    )
endif()
