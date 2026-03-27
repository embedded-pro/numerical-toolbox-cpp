add_library(numerical.simulator.estimators.rls.view STATIC)

set_target_properties(numerical.simulator.estimators.rls.view PROPERTIES AUTOMOC ON)

target_include_directories(numerical.simulator.estimators.rls.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
)

target_link_libraries(numerical.simulator.estimators.rls.view PUBLIC
    numerical.simulator.estimators.rls.application
    numerical.simulator.widgets
    Qt6::Widgets
)

target_sources(numerical.simulator.estimators.rls.view PRIVATE
    RlsConfigurationPanel.hpp
    RlsConfigurationPanel.cpp
    RlsMainWindow.hpp
    RlsMainWindow.cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(numerical.simulator.estimators.rls.view PRIVATE -Wno-error)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(numerical.simulator.estimators.rls.view PRIVATE /W4 /wd4244 /wd4100)
endif()
