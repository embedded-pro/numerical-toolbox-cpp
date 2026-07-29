add_library(numerical.integration_test.steps OBJECT)

target_sources(numerical.integration_test.steps PRIVATE
    PidControllerSteps.cpp
)

target_link_libraries(numerical.integration_test.steps PRIVATE
    cucumber_cpp.runner
    numerical.controllers.implementations
)
