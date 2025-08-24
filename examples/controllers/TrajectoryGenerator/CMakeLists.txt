add_executable(TrajectoryGenerator TrajectoryGenerator.cpp)

target_link_libraries(TrajectoryGenerator
    numerical.controllers
)

target_include_directories(TrajectoryGenerator PRIVATE
    ${CMAKE_SOURCE_DIR}
)
