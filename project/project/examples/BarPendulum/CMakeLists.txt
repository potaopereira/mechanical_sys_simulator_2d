
set(APPLICATION_NAME BarPendulumAPP)

set(
    SRCS
    src/main.cpp
    src/BarPendulum.cpp
)

add_executable(
    ${APPLICATION_NAME}
    ${SRCS}
)

target_include_directories(
    ${APPLICATION_NAME}
    PUBLIC include
)

target_link_libraries(
    ${APPLICATION_NAME}
    PUBLIC MSInterface
    PUBLIC MSSolver
    PUBLIC MSViewer
)

# add_launch_json(_name _program _args _cwd)
add_launch_json(
    # _name
    "${APPLICATION_NAME}"
    # _program
    "${CMAKE_CURRENT_BINARY_DIR}/${APPLICATION_NAME}"
    # _args
    "${CMAKE_CURRENT_BINARY_DIR}/dfunctions.cpp"
    # _cwd
    "${CMAKE_CURRENT_BINARY_DIR}"
)


set(APPLICATION_NAME BarPendulum)

set(
    SRCS
    src/BarPendulum.cpp
    src/dfunctions.cpp
    src/BarPendulumSolver.cpp
    src/BarPendulumViewer.cpp
)

add_library(
    ${APPLICATION_NAME}
    ${SRCS}
)

target_include_directories(
    ${APPLICATION_NAME}
    PUBLIC include
)

target_link_libraries(
    ${APPLICATION_NAME}
    PUBLIC MSInterface
    PUBLIC MSSolver
    PUBLIC MSViewer
)
