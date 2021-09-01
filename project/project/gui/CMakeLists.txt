set(PROJECT_APPLICATION_NAME gui)

add_compile_options(-Wall -Wextra -pedantic -Werror)

# this is required with QOBJECT, "signals" and "slots"
set(CMAKE_AUTOMOC ON)

set(SRC_FILES
    src/main.cpp
    src/mainwindow.cpp
    src/colorbutton.cpp
    src/mechanical_system_options.cpp
    src/addMS.cpp
    src/global_options.cpp

)

set(HEADER_FILES
    include/private/mainwindow.h
    include/private/colorbutton.h
    include/private/mechanical_system_options.h
    include/private/addMS.h
    include/private/global_options.h
)

add_executable(
    ${PROJECT_APPLICATION_NAME}
    ${SRC_FILES}
    ${HEADER_FILES}
)

target_include_directories(
    ${PROJECT_APPLICATION_NAME}
    PUBLIC include
    PRIVATE include/private
)

find_package(Qt5 COMPONENTS
    Core
    Gui
    Widgets
    REQUIRED)

target_link_libraries(${PROJECT_APPLICATION_NAME}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    MSInterface
    yaml-cpp
    ExampleMS
)

# add_launch_json(_name _program _args _cwd)
add_launch_json(
    # _name
    "${PROJECT_APPLICATION_NAME}"
    # _program
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_APPLICATION_NAME}"
    # _args
    ""
    # _cwd
    "${CMAKE_CURRENT_BINARY_DIR}"
)

configure_file(
    config/config.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config.yaml
    @ONLY
)