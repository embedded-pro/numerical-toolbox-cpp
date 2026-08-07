add_library(platform_qemu_startup STATIC
    FaultHandler.cpp
    Startup.cpp
    DefaultInit.cpp
)

set_target_properties(platform_qemu_startup PROPERTIES COMPILE_WARNING_AS_ERROR Off)

target_include_directories(platform_qemu_startup PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_options(platform_qemu_startup INTERFACE
    -T ${CMAKE_CURRENT_SOURCE_DIR}/mps2.ld
    -Wl,-u,Reset_Handler
    -Wl,-u,FaultDispatch
    -Wl,-u,main
)
