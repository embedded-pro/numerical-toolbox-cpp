function(numerical_toolbox_enable_embedded_optimizations)
    cmake_parse_arguments(ARG "" "" "TARGETS" ${ARGN})

    if (NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        return()
    endif()

    set(options "$<$<COMPILE_LANGUAGE:C,CXX>:-ffast-math;-fno-finite-math-only>")

    if (ARG_TARGETS)
        foreach(target ${ARG_TARGETS})
            target_compile_options(${target} PRIVATE "${options}")
        endforeach()
    else()
        add_compile_options("${options}")
    endif()
endfunction()
