add_library(dsp.math_test_helper INTERFACE)
emil_build_for(dsp.math_test_helper BOOL CONTROL_TOOLBOX_BUILD_TESTS)

target_link_libraries(dsp.math_test_helper INTERFACE
    gmock
    dsp.math
)

target_sources(dsp.math_test_helper PRIVATE
    AdvancedFunctionsStub.hpp
    HyperbolicFunctionsStub.hpp
    SingleInstructionMultipleDataStub.hpp
    TrigonometricFunctionsStub.hpp
)
