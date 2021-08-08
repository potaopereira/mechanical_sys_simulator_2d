set(PROJECT_NAME RB2D)

set(
    PUB_HEADERS
    include/rb2d/rb2d.hpp
)

set(
    SRCS
    src/rb2d.cpp
)

add_library(
    ${PROJECT_NAME}
    ${SRCS}
)

include_directories(
    ${PROJECT_NAME}
    PUBLIC include/
)

# add_launch_json(_name _program _args _cwd)
add_launch_json(
    # _name
    "${PROJECT_NAME}"
    # _program
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    # _args
    ""
    # _cwd
    "${CMAKE_CURRENT_BINARY_DIR}"
)

add_subdirectory(unit-test)