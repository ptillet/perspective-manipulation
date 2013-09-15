project(PERSPECTIVE_MANIPULATION)

cmake_minimum_required(VERSION 2.6)

set(cmake_verbose_makefile on)

set(EXECUTABLE_OUTPUT_PATH ${PERSPECTIVE_MANIPULATION_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PERSPECTIVE_MANIPULATION_BINARY_DIR}/lib)

add_subdirectory(src)
