
set(APPLICATION_NAME ExampleMSAPP)

set(
    SRCS
    src/main.cpp
    src/examplems.cpp
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




# message("CMAKE_COMMAND is "${CMAKE_COMMAND})
# add_custom_target(
#     ${CMAKE_CURRENT_BINARY_DIR}/dfunctions.cpp
#     ${CMAKE_CURRENT_BINARY_DIR}/dfunctions.cpp
# )

# add_custom_command(OUTPUT dfunctions.cpp
#   COMMAND ${CMAKE_COMMAND} -S 
#   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/"
#   DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/template.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/include/template/template.hpp"
#   COMMENT "Making functions for template mechanical system"
#   VERBATIM
# )








set(APPLICATION_NAME ExampleMS)

set(
    SRCS
    src/examplems.cpp
    src/dfunctions.cpp
    src/examplemssolver.cpp
    src/examplemsviewer.cpp
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
