numerical_add_header_library(numerical.controllers.implementations)

target_include_directories(numerical.controllers.implementations ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers.implementations ${NUMERICAL_VISIBILITY}
    numerical.controllers.interfaces
    numerical.filters.active
    numerical.solver
)

target_sources(numerical.controllers.implementations PRIVATE
    BangBangHysteresis.hpp
    DeadbeatControl.hpp
    Feedforward2Dof.hpp
    GainScheduledController.hpp
    IntegralStateFeedbackLqi.hpp
    LeadLagCompensator.hpp
    Lqg.hpp
    Lqr.hpp
    LuenbergerObserver.hpp
    Mpc.hpp
    PidIncremental.hpp
    SaturationRateLimiter.hpp
)

numerical_add_coverage_sources(numerical.controllers.implementations
    BangBangHysteresis.cpp
    DeadbeatControl.cpp
    Feedforward2Dof.cpp
    GainScheduledController.cpp
    IntegralStateFeedbackLqi.cpp
    LeadLagCompensator.cpp
    Lqg.cpp
    Lqr.cpp
    LuenbergerObserver.cpp
    Mpc.cpp
    PidIncremental.cpp
    SaturationRateLimiter.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
