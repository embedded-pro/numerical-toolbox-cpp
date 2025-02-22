add_library(numerical.math_test_helper INTERFACE)
emil_build_for(numerical.math_test_helper BOOL NUMERICAL_TOOLBOX_BUILD_TESTS)

target_link_libraries(numerical.math_test_helper INTERFACE
    gmock
    numerical.math
)

target_sources(numerical.math_test_helper PRIVATE
    AdvancedFunctionsStub.hpp
    HyperbolicFunctionsStub.hpp
    SingleInstructionMultipleDataStub.hpp
    TrigonometricFunctionsStub.hpp
)
