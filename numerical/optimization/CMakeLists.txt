numerical_add_header_library(numerical.optimization)

target_include_directories(numerical.optimization ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.optimization ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.optimization PRIVATE
    Adam.hpp
    BayesianOptimization.hpp
    DampingPolicy.hpp
    GradientDescent.hpp
    LeastSquaresProblem.hpp
    LevenbergMarquardt.hpp
    ObjectiveFunction.hpp
    Optimizer.hpp
    Sgd.hpp
    StepOptimizer.hpp
)

numerical_add_coverage_sources(numerical.optimization
    Adam.cpp
    BayesianOptimization.cpp
    GradientDescent.cpp
    LevenbergMarquardt.cpp
    Sgd.cpp
)

add_subdirectory(test)
