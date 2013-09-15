set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(FILES mode_manager window_handler user_infos line interpolate)

foreach(file ${FILES})
    set(SRCS ${SRCS} ${SRC_DIR}/${file}.cpp)
endforeach()

add_library(ui ${SRCS})
