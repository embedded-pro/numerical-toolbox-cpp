add_library(numerical.kinematics ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.kinematics PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.kinematics PUBLIC
    numerical.dynamics
    numerical.math
)

target_sources(numerical.kinematics PRIVATE
    ForwardKinematics.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.kinematics PRIVATE
        ForwardKinematics.cpp
    )
endif()

add_subdirectory(test)
