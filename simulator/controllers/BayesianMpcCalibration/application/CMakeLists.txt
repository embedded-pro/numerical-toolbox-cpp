add_library(numerical.simulator.controllers.bayesian_mpc_calibration.application STATIC)

target_include_directories(numerical.simulator.controllers.bayesian_mpc_calibration.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.controllers.bayesian_mpc_calibration.application PUBLIC
    numerical.controllers.implementations
    numerical.estimators.offline
    numerical.filters.active
    numerical.optimization
    numerical.solver
)

target_sources(numerical.simulator.controllers.bayesian_mpc_calibration.application PRIVATE
    BayesianMpcCalibrationSimulator.cpp
    BayesianMpcCalibrationSimulator.hpp
    DoubleIntegratorPlant.cpp
    DoubleIntegratorPlant.hpp
    EmKfPipeline.cpp
    EmKfPipeline.hpp
    MpcBoCalibrator.cpp
    MpcBoCalibrator.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.controllers.bayesian_mpc_calibration.application PRIVATE
        -Wno-error
    )
endif()
