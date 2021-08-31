set(PROJECT_NAME MSSolver)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GSL REQUIRED)

set(
    PUB_HEADERS
    include/rb2d/mssolverimpl.hpp
    include/rb2d/util.hpp
)

set(
    SRCS
    src/util.cpp
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
    PUBLIC MSInterface
    PUBLIC SymbolicCpp
    PUBLIC Eigen3::Eigen
    PUBLIC GSL::gsl
    PUBLIC GSL::gslcblas
)

# # add_launch_json(_name _program _args _cwd)
# add_launch_json(
#     # _name
#     "${PROJECT_NAME}"
#     # _program
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
#     # _args
#     ""
#     # _cwd
#     "${CMAKE_CURRENT_BINARY_DIR}"
# )

add_subdirectory(unit-test)