cmake_minimum_required(VERSION 3.24)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CONTROL_TOOLBOX_STANDALONE On)
endif()

option(CMAKE_COMPILE_WARNING_AS_ERROR "Enable warnings-as-error" On)
option(CONTROL_TOOLBOX_INCLUDE_DEFAULT_INIT "Include default initialization code; turn off when providing custom initialization" On)
option(CONTROL_TOOLBOX_BUILD_EXAMPLES "Enable build of the examples" Off)
option(CONTROL_TOOLBOX_BUILD_TESTS "Enable build of the examples" Off)

if (CONTROL_TOOLBOX_BUILD_TESTS)
    # CTest cannot be included before the first project() statement, but Embedded Infastructure
    # Library needs to see that test utilities need to be built.
    set(BUILD_TESTING On)
endif()

add_definitions(-DEMIL_ENABLE_TRACING=1)

if (CONTROL_TOOLBOX_STANDALONE)
    include(FetchContent)

    FetchContent_Declare(
        emil
        GIT_REPOSITORY https://gitlab.com/embedded-library/infrastructure/embedded-infra-lib.git
        GIT_TAG        dc2b87ee100366b4141012d0c86680cf2c7a311b # Unreleased
    )

    FetchContent_MakeAvailable(emil)

    if (TARGET_MCU_VENDOR STREQUAL st)
        FetchContent_Declare(
            halst
            GIT_REPOSITORY https://gitlab.com/embedded-library/infrastructure/hal-st.git
            GIT_TAG        6563a5806afc5a593a96a2be53329b59ab40b5c3 # unreleased
        )

        FetchContent_MakeAvailable(halst)
    endif()
endif()

project(control_toolbox LANGUAGES C CXX ASM VERSION 0.0.1) # x-release-please-version

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set_directory_properties(PROPERTY USE_FOLDERS ON)

# When building usb-host by itself do not exclude any targets from all
# Libraries will only be excluded from all when usb-host is a consumed as a dependency.
if (CONTROL_TOOLBOX_STANDALONE)
    set(CONTROL_TOOLBOX_EXCLUDE_FROM_ALL "")
else()
    set(CONTROL_TOOLBOX_EXCLUDE_FROM_ALL "EXCLUDE_FROM_ALL")
endif()

if (CONTROL_TOOLBOX_BUILD_TESTS)
    include(CTest)
    emil_enable_testing()
endif()

add_subdirectory(controllers)
add_subdirectory(examples)

emil_clangformat_directories(usb_host DIRECTORIES .)

if (CONTROL_TOOLBOX_STANDALONE)
    emil_folderize_all_targets()
endif()
