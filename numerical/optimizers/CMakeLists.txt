add_library(numerical.optmizers ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.optmizers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.optmizers PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.optmizers PRIVATE
    BinaryCrossEntropy.hpp
    CrossEntropy.hpp
    GradientDescent.cpp
    GradientDescent.hpp
    Loss.hpp
    Optimizer.hpp
)

add_subdirectory(test)
