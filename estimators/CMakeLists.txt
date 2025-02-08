add_library(dsp.estimators ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.estimators PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.estimators PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.estimators PRIVATE
    Estimator.hpp
    LinearRegression.cpp
    LinearRegression.hpp
    YuleWalker.hpp
)

add_subdirectory(test)
