function(numerical_add_qemu_test target)
    if(NOT DEFINED QEMU_MACHINE)
        return()
    endif()
    target_link_libraries(${target} PRIVATE platform_qemu_startup)
    target_link_libraries(${target} PRIVATE -lrdimon)
    add_test(
        NAME qemu.${target}
        COMMAND qemu-system-arm
            -machine ${QEMU_MACHINE}
            -nographic
            -semihosting-config enable=on,target=native
            -kernel $<TARGET_FILE:${target}>
    )
    set_tests_properties(qemu.${target} PROPERTIES TIMEOUT 120)
endfunction()
