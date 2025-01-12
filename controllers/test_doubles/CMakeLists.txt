add_library(dsp.controllers_test_helper INTERFACE)
emil_build_for(dsp.controllers_test_helper BOOL CONTROL_TOOLBOX_BUILD_TESTS)

target_link_libraries(dsp.controllers_test_helper INTERFACE
    gmock
    dsp.math
)

target_sources(dsp.controllers_test_helper PRIVATE
    NormalizedAngles.hpp
    Tolerance.hpp
)
