# - Find fftw (Fast Fourier Transform) libraries
# This module checks if fftw libraries are installed and determines where the
# include files and libraries are. This code sets the following variables:
#
#  FFTW_FOUND         - have the FFTW libraries been found
#  FFTW_LIBRARIES     - names of the FFTW libraries
#  FFTW_INCLUDE_PATH  - path to the FFTW includes
#  FFTW_LIBRARY_PATH  - path to the FFTW libraries
#  FFTW_RESOURCE_PATH - path to the FFTW resources
#

set(FFTW_FOUND TRUE)

# Find include files and include paths

find_path(FFTW_INCLUDE_PATH NAMES fftw3.h)

if (NOT FFTW_INCLUDE_PATH)
    set(FFTW_FOUND FALSE)
    set(FFTW_ERROR_REASON "${FFTW_ERROR_REASON} Includes not found.")
else(NOT FFTW_INCLUDE_PATH)
    message(STATUS "FFTW include path ${FFTW_INCLUDE_PATH}")
endif (NOT FFTW_INCLUDE_PATH)

# Find libraries and library paths

set(FFTW_LIBRARYNAME "fftw3")
find_library(FFTW_LIBRARY NAMES ${FFTW_LIBRARYNAME})
 
if (NOT FFTW_LIBRARY)
    set(FFTW_FOUND FALSE)
    set(FFTW_ERROR_REASON
        "${FFTW_ERROR_REASON} Library ${FFTW_LIBRARYNAME} not found.")
else(NOT FFTW_LIBRARY)
    get_filename_component(FFTW_LIBRARY_PATH ${FFTW_LIBRARY} PATH CACHE)
    list(APPEND FFTW_LIBRARIES ${FFTW_LIBRARYNAME})
endif (NOT FFTW_LIBRARY)

if (FFTW_FOUND)
    message(STATUS "FFTW libraries ${FFTW_LIBRARIES}")
    message(STATUS "FFTW library path ${FFTW_LIBRARY_PATH}")
endif (FFTW_FOUND)

if (NOT FFTW_FOUND)
    if(FFTW_FIND_REQUIRED)
        message(SEND_ERROR "Unable to find FFTW.\n${FFTW_ERROR_REASON}")
    else(FFTW_FIND_REQUIRED)
        message(STATUS "Unable to find FFTW: ${FFTW_ERROR_REASON}")
    endif(FFTW_FIND_REQUIRED)
else(NOT FFTW_FOUND)
    ADD_DEFINITIONS(-D__FFTW_FOUND__)
    link_directories   (${FFTW_LIBRARY_PATH})
    include_directories(${FFTW_INCLUDE_PATH})
endif(NOT FFTW_FOUND)
