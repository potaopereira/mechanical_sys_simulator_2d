set(PROJECT_APPLICATION_NAME MSViewerGui)

# this is required with QOBJECT, "signals" and "slots"
set(CMAKE_AUTOMOC ON)

set(SRC_FILES
    src/main.cpp
    src/mainwindow.cpp
)

set(HEADER_FILES
    include/msviewergui/mainwindow.h
)

add_executable(
    ${PROJECT_APPLICATION_NAME}
    ${SRC_FILES}
    ${HEADER_FILES}
)

target_include_directories(
    ${PROJECT_APPLICATION_NAME}
    PUBLIC include
    PRIVATE include/msviewergui
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
    MSViewer
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