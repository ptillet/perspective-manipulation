if (UMFPACK_INCLUDES AND UMFPACK_LIBRARIES)
set(UMFPACK_FIND_QUIETLY TRUE)
endif (UMFPACK_INCLUDES AND UMFPACK_LIBRARIES)

find_package(BLAS)
if(BLAS_FOUND)
find_path(UMFPACK_INCLUDES NAMES umfpack.h PATHS $ENV{UMFPACKDIR} ${INCLUDE_INSTALL_DIR} PATH_SUFFIXES suitesparse)
find_library(UMFPACK_LIBRARIES umfpack PATHS $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
if(UMFPACK_LIBRARIES)
get_filename_component(UMFPACK_LIBDIR ${UMFPACK_LIBRARIES} PATH)
endif(UMFPACK_LIBRARIES)
endif(BLAS_FOUND)

mark_as_advanced(UMFPACK_INCLUDES UMFPACK_LIBRARIES)