project(create_vscode_launch)

cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Debug)
set(VSCODEIN ${CMAKE_SOURCE_DIR}/vscode_in)

set_property(GLOBAL PROPERTY _LAUNCHS "")
function(add_launch_json _name _program _args _cwd)
    set(launch launch_${_name}.json)
    set(name ${_name})
    set(program ${_program})
    set(args ${_args})
    set(cwd ${_cwd})
    configure_file(
        ${VSCODEIN}/configuration.in
        ${CMAKE_BINARY_DIR}/.vscode/${launch}
    )
    set_property(GLOBAL APPEND PROPERTY _LAUNCHS launch_${name}.json)
endfunction()

# add_subdirectory(rb2d)
add_subdirectory(SymbolicC++3-3.35)
add_subdirectory(msinterface)
add_subdirectory(msviewer)
add_subdirectory(mssolver)
add_subdirectory(examples)
add_subdirectory(gui)
# add_subdirectory(minimizer)
# add_subdirectory(qt_try)




get_property(LAUNCHS GLOBAL PROPERTY _LAUNCHS)
message("Launchs = " ${LAUNCHS})

set(configurations)
foreach(launch ${LAUNCHS})
    file(READ ${CMAKE_BINARY_DIR}/.vscode/${launch} configuration)
    string(APPEND configurations ${configuration})
endforeach()

# withou @ONLY ${workspaceFolder} is replaced
configure_file(
    vscode_in/launch.json.in
    ${CMAKE_BINARY_DIR}/.vscode/launch.json
    @ONLY
)

# withou @ONLY ${workspaceFolder} is replaced
configure_file(
    vscode_in/tasks.json.in
    ${CMAKE_BINARY_DIR}/.vscode/tasks.json
    @ONLY
)

configure_file(
    vscode_in/workspace.code-workspace
    ${CMAKE_BINARY_DIR}/workspace.code-workspace
    @ONLY
)