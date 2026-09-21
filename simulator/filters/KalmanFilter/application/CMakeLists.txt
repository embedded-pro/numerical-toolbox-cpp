add_library(numerical.simulator.filters.kalman.application STATIC)

target_include_directories(numerical.simulator.filters.kalman.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.simulator.filters.kalman.application PUBLIC
    numerical.filters.active
    numerical.simulator.shell
    ui.model
)

target_sources(numerical.simulator.filters.kalman.application PRIVATE
    KalmanFilterSimulator.cpp
    KalmanFilterSimulator.hpp
    KalmanForm.cpp
    KalmanForm.hpp
    PendulumPlant.cpp
    PendulumPlant.hpp
)

if(MSVC)
    target_compile_options(numerical.simulator.filters.kalman.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(numerical.simulator.filters.kalman.application PRIVATE
        -Wno-error
    )
endif()

if (NUMERICAL_TOOLBOX_BUILD_TESTS)
    add_subdirectory(test)
endif()
