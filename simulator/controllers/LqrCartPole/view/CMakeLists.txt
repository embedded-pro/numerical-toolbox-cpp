add_library(numerical.simulator.controllers.lqr.view STATIC)

target_include_directories(numerical.simulator.controllers.lqr.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.lqr.view PUBLIC
    numerical.simulator.controllers.lqr.application
    numerical.simulator.utils
    Qt6::Widgets
)

set_target_properties(numerical.simulator.controllers.lqr.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.controllers.lqr.view PRIVATE
    CartPoleWidget.cpp
    CartPoleWidget.hpp
    LqrConfigurationPanel.cpp
    LqrConfigurationPanel.hpp
    LqrEvaluationWidget.cpp
    LqrEvaluationWidget.hpp
    LqrMainWindow.cpp
    LqrMainWindow.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.lqr.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.lqr.view PRIVATE
        -Wno-error
    )
endif()
