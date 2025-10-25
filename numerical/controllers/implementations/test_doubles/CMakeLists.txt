add_library(numerical.controllers_test_helper INTERFACE)
emil_build_for(numerical.controllers_test_helper BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)

target_link_libraries(numerical.controllers_test_helper INTERFACE
    gmock
    numerical.math
)

target_sources(numerical.controllers_test_helper PRIVATE
    NormalizedAngles.hpp
)
