numerical_add_header_library(numerical.motion_profiles)

target_include_directories(numerical.motion_profiles ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.motion_profiles ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.motion_profiles PRIVATE
    MotionProfileTypes.hpp
    PolynomialProfile.hpp
    SCurveProfile.hpp
    TrapezoidalProfile.hpp
)

numerical_add_coverage_sources(numerical.motion_profiles
    PolynomialProfile.cpp
    SCurveProfile.cpp
    TrapezoidalProfile.cpp
)

add_subdirectory(test)
