
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Umfpack)
include_directories(${UMFPACK_INCLUDES} include)

set(CMAKE_CXX_FLAGS "-g -Wall")

#Hack to make CMake conscious about the headers
file( GLOB_RECURSE DUMMY_SRC *.h *.hpp)
add_custom_target( Dummy SOURCES ${DUMMY_SRC} )

add_subdirectory(cpp)
