# view of two-dimensional rigid body
set(APPLICATION_NAME MSViewer)

# find_library( EIGEN_LIBRARY NAMES Eigen)
# message("EIGEN_LIBRARY = " ${EIGEN_LIBRARY})
# Since Eigen3 is completely header only, all you ever need is the path to the include directory. And this one, you are already defining manually anyway. So there is no real need for a FindEigen3.cmake or FIND_PACKAGE call.
# Simply use: INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(GSL REQUIRED)

# this is required with QOBJECT, "signals" and "slots"
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS
    Core
    Widgets
    Gui
    REQUIRED
)

set(
    PUB_HEADERS
    include/msviewer/rbview.hpp
    include/msviewer/rbviewwvectors.hpp
    include/msviewer/rbwithboundary.hpp
    include/msviewer/viewerimpl.hpp
    # include/view/combined.hpp
)

set(
    SRCS
    src/rbview.cpp
    src/rbviewwvectors.cpp
    src/rbwithboundary.cpp
    # src/combined.cpp
)

add_library(
    ${APPLICATION_NAME}
    ${SRCS}
    ${PUB_HEADERS}
)

target_include_directories(
    ${APPLICATION_NAME}
    PUBLIC include
)

target_link_libraries(
    ${APPLICATION_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    Eigen3::Eigen
    MSInterface
)

add_subdirectory(unit-test)