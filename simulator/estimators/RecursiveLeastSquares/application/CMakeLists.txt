add_library(numerical.simulator.estimators.rls.application STATIC)

target_include_directories(numerical.simulator.estimators.rls.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
)

target_link_libraries(numerical.simulator.estimators.rls.application PUBLIC
    numerical.estimators.online
)

target_sources(numerical.simulator.estimators.rls.application PRIVATE
    RlsSimulator.hpp
    RlsSimulator.cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(numerical.simulator.estimators.rls.application PRIVATE -Wno-error)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(numerical.simulator.estimators.rls.application PRIVATE /W4 /wd4244 /wd4100)
endif()
