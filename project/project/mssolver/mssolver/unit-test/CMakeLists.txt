set(PROJECT_NAME MSSolverTest)


find_library( CPPUNIT_LIBRARY_DEBUG NAMES cppunit)
message("CPPUNIT_LIBRARY_DEBUG = " ${CPPUNIT_LIBRARY_DEBUG})

add_executable(
    run${PROJECT_NAME}
    src/factoryRegistry.cpp
    src/testClass.cpp
)

target_link_libraries(
    run${PROJECT_NAME}
    PUBLIC cppunit
)

# add_launch_json(_name _program _args _cwd)
add_launch_json(
    # _name
    "${PROJECT_NAME}"
    # _program
    "${CMAKE_CURRENT_BINARY_DIR}/run${PROJECT_NAME}"
    # _args
    ""
    # _cwd
    "${CMAKE_CURRENT_BINARY_DIR}"
)