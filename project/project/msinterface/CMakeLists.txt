set(PROJECT_NAME MSInterface)

add_compile_options(-Wall -Wextra -pedantic -Werror)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Qt5 COMPONENTS
    Gui
    Widgets
    REQUIRED
)

set(
    PUB_HEADERS
    include/msinterface/param.hpp
    include/msinterface/solver.hpp
    include/msinterface/viewer.hpp
    include/msinterface/interface.hpp
    include/msinterface/exceptions.hpp
)

set(
    SRCS
    src/param.cpp
    src/solver.cpp
    src/viewer.cpp
)

add_library(
    ${PROJECT_NAME}
    ${SRCS}
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC include
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC Eigen3::Eigen
    Qt5::Gui
    Qt5::Widgets
)