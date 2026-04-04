add_library(numerical.simulator.controllers.bayesian_mpc_calibration.view STATIC)

target_include_directories(numerical.simulator.controllers.bayesian_mpc_calibration.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.bayesian_mpc_calibration.view PUBLIC
    numerical.simulator.controllers.bayesian_mpc_calibration.application
    numerical.simulator.widgets
    Qt6::Widgets
)

set_target_properties(numerical.simulator.controllers.bayesian_mpc_calibration.view PROPERTIES
    AUTOMOC ON
)

target_sources(numerical.simulator.controllers.bayesian_mpc_calibration.view PRIVATE
    BayesianMpcCalibrationView.cpp
    BayesianMpcCalibrationView.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration.view PRIVATE
        -Wno-error
    )
endif()
