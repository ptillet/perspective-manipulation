
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(FILES mesh vertex)

foreach(file ${FILES})
    set(SRCS ${SRCS} ${SRC_DIR}/${file}.cpp)
endforeach()

add_library(optimization ${SRCS})
target_link_libraries(optimization umfpack)
