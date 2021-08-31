set(APPLICATION_NAME MSViewerTest)

find_library( CPPUNIT_LIBRARY_DEBUG NAMES cppunit)
# message("CPPUNIT_LIBRARY_DEBUG = " ${CPPUNIT_LIBRARY_DEBUG})

add_executable(
    run${APPLICATION_NAME}
    src/factoryRegistry.cpp # main
    src/testClass.cpp
)

target_link_libraries(
    run${APPLICATION_NAME}
    PUBLIC cppunit
)

# add_launch_json(_name _program _args _cwd)
add_launch_json(
    # _name
    "${APPLICATION_NAME}"
    # _program
    "${CMAKE_CURRENT_BINARY_DIR}/run${APPLICATION_NAME}"
    # _args
    ""
    # _cwd
    "${CMAKE_CURRENT_BINARY_DIR}"
)