set(PROJECT_NAME RB2DTest)


find_library( CPPUNIT_LIBRARY_DEBUG NAMES cppunit)
message("CPPUNIT_LIBRARY_DEBUG = " ${CPPUNIT_LIBRARY_DEBUG})

add_executable(
    run${PROJECT_NAME}
    src/runTestFactoryRegistry.cpp
    src/runTestFactoryRegistryInput.cpp
)

target_link_libraries(
    run${PROJECT_NAME}
    PUBLIC RB2D
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