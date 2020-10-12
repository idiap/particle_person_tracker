# - Find CMotion2D library
# This module finds CMotion2D library and its header files.
# This code sets the following variables:
#
#  CMotion2D_FOUND         - have the CTU face detector libs been found
#  CMotion2D_LIBRARIES     - names of the CTU face detector libraries
#  CMotion2D_INCLUDE_PATH  - path to the CTU face detector includes
#  CMotion2D_LIBRARY_PATH  - path to the CTU face detector libraries
#

set(CMotion2D_FOUND TRUE)

set(CMotion2D_LIBRARYNAME "cmotion2d")

find_library(CMotion2D_LIBRARY NAMES ${CMotion2D_LIBRARYNAME})

# Find libraries and library paths
 
if (NOT CMotion2D_LIBRARY)
    set(CMotion2D_FOUND FALSE)
    set(CMotion2D_ERROR_REASON
        "${CMotion2D_ERROR_REASON} Libraries not found.")
else(NOT CMotion2D_LIBRARY)
    get_filename_component(CMotion2D_LIBRARY_PATH
        ${CMotion2D_LIBRARY} PATH CACHE)
    set(CMotion2D_LIBRARIES ${CMotion2D_LIBRARYNAME})
    message(STATUS "CMotion2D libraries ${CMotion2D_LIBRARIES}")
    message(STATUS "CMotion2D library path ${CMotion2D_LIBRARY_PATH}")
    get_filename_component(CMotion2D_BASE_LIB_PATH
        ${CMotion2D_LIBRARY_PATH} PATH)
endif (NOT CMotion2D_LIBRARY)

# Find include files and include paths

find_path(CMotion2D_INCLUDE_PATH NAMES cmotion2d/CMotion2D.h)

if (NOT CMotion2D_INCLUDE_PATH)
    set(CMotion2D_FOUND FALSE)
    set(CMotion2D_ERROR_REASON
        "${CMotion2D_ERROR_REASON} Includes not found.")
else(NOT CMotion2D_INCLUDE_PATH)
    get_filename_component(CMotion2D_BASE_INC_PATH
        ${CMotion2D_INCLUDE_PATH} PATH)
    message(STATUS "CMotion2D includes ${CMotion2D_INCLUDE_PATH}")
endif (NOT CMotion2D_INCLUDE_PATH)

if (NOT CMotion2D_FOUND)
    if(CMotion2D_FIND_REQUIRED)
        message(SEND_ERROR "Unable to find CMotion2D.\n${CMotion2D_ERROR_REASON}")
    else(CMotion2D_FIND_REQUIRED)
        message(STATUS "Unable to find CMotion2D: ${CMotion2D_ERROR_REASON}")
    endif(CMotion2D_FIND_REQUIRED)
else(NOT CMotion2D_FOUND)
    ADD_DEFINITIONS(-D__CMOTION2D_FOUND__)
    link_directories   (${CMotion2D_LIBRARY_PATH})
    include_directories(${CMotion2D_INCLUDE_PATH})
endif(NOT CMotion2D_FOUND)
