set(CCR_FETCH_DEPS On CACHE BOOL "" FORCE)
set(CCR_BUILD_TESTS Off CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    cucumber_cpp
    GIT_REPOSITORY https://github.com/philips-software/amp-cucumber-cpp-runner.git
    GIT_TAG        v4.1.0
)
FetchContent_MakeAvailable(cucumber_cpp)

# cucumber_gherkin uses std::wstring_convert which is deprecated in C++17/GCC15.
# Suppress -Werror=deprecated-declarations for this third-party target only.
if (TARGET cucumber_cpp.library.api)
    target_compile_options(cucumber_cpp.library.api PRIVATE -Wno-deprecated-declarations)
endif()

add_executable(numerical.integration_test)

target_sources(numerical.integration_test PRIVATE
    Main.cpp
)

target_link_libraries(numerical.integration_test PRIVATE
    cucumber_cpp
    numerical.integration_test.steps
)

add_subdirectory(steps)

add_test(NAME numerical.integration_test
    COMMAND numerical.integration_test --format pretty -- ${CMAKE_CURRENT_LIST_DIR}/features
)
