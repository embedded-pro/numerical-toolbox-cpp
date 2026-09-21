# Deliberately two targets. The application layer of every simulator builds its form here and must
# stay Qt-free so it can be tested with no display; only the entry point needs a QApplication.
add_library(numerical.simulator.shell INTERFACE)

target_include_directories(numerical.simulator.shell INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_sources(numerical.simulator.shell INTERFACE
    Guard.hpp
    SignalComponents.hpp
)

target_link_libraries(numerical.simulator.shell INTERFACE
    numerical.simulator.utils
    ui.model
    ui.shell
)

add_library(numerical.simulator.shell.qt INTERFACE)

target_sources(numerical.simulator.shell.qt INTERFACE
    AppRunner.hpp
)

target_link_libraries(numerical.simulator.shell.qt INTERFACE
    numerical.simulator.shell
    ui.backend.qt
)
