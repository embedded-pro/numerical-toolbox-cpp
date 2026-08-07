target_sources(numerical.math PRIVATE
    MathArm.hpp
)

if(NUMERICAL_TOOLBOX_ENABLE_CUSTOM_MATH AND CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    target_compile_options(numerical.math INTERFACE
        -include ${CMAKE_CURRENT_LIST_DIR}/MathArm.hpp
    )
endif()
