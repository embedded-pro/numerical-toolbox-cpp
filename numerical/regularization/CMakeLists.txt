numerical_add_header_library(numerical.regularization)

target_include_directories(numerical.regularization ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.regularization ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.regularization PRIVATE
    L1.hpp
    L2.hpp
    Regularization.hpp
)

numerical_add_coverage_sources(numerical.regularization
    L1.cpp
    L2.cpp
)

add_subdirectory(test)
