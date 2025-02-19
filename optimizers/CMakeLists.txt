add_library(dsp.optmizers ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.optmizers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.optmizers PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.optmizers PRIVATE
    BinaryCrossEntropy.hpp
    CrossEntropy.hpp
    GradientDescent.cpp
    GradientDescent.hpp
    Loss.hpp
    Optimizer.hpp
)

add_subdirectory(test)
