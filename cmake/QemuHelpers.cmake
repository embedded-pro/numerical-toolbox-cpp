function(numerical_link_qemu_runtime target)
    if(NOT EMIL_BUILD_QEMU)
        return()
    endif()
    target_link_libraries(${target} PRIVATE
        hal.cortex_m
        hal.cortex_m.runtime
        hal.qemu.syscalls
        hal.qemu.default_init
        hal.qemu.sync
        hal.qemu.cortex
        gmock_main
    )
    # newlib-nano's printf has no float conversions unless _printf_float is linked. Without them the
    # vsnprintf behind std::ostream << float returns a bogus length, libstdc++ allocas that many bytes,
    # and the first failure message that prints a float wraps the stack pointer and locks up the core.
    target_link_options(${target} PRIVATE "LINKER:--undefined=_printf_float")
endfunction()
