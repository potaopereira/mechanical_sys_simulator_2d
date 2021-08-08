set(PROJECT_NAME RB2DTest)


find_library( CPPUNIT_LIBRARY_DEBUG NAMES cppunit)
message("CPPUNIT_LIBRARY_DEBUG = " ${CPPUNIT_LIBRARY_DEBUG})

add_executable(
    run_TestFactoryRegistry
    src/runTestFactoryRegistry.cpp
    src/runTestFactoryRegistryInput.cpp
)

target_link_libraries(
    run_TestFactoryRegistry
    PUBLIC RB2D
    PUBLIC cppunit
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