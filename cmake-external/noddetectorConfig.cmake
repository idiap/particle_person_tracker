# - Find nod detector module
# This module checks if nod detector is installed and determines where the
# include files and libraries are. This code sets the following variables:
#
#  NodDetector_FOUND         - have the NodDetector libs been found
#  NodDetector_LIBRARIES     - names of the NodDetector libraries
#  NodDetector_INCLUDE_PATH  - path to the NodDetector includes
#  NodDetector_LIBRARY_PATH  - path to the NodDetector libraries
#  NodDetector_RESOURCE_PATH - path to the NodDetector resources
#

if (NOT NONODDETECTOR)
    
    set(NodDetector_FOUND TRUE)
    
    # Find include files and include paths
    
    find_path(NodDetector_INCLUDE_PATH
        NAMES noddetector/nd_NodDetector.h)
    
    if (NOT NodDetector_INCLUDE_PATH)
        set(NodDetector_FOUND FALSE)
        set(NodDetector_ERROR_REASON
            "${NodDetector_ERROR_REASON} Includes not found.")
    else(NOT NodDetector_INCLUDE_PATH)
        get_filename_component(NodDetector_BASE_INC_PATH
            ${NodDetector_INCLUDE_PATH} PATH)
        message(STATUS "NodDetector includes ${NodDetector_INCLUDE_PATH}")
    endif (NOT NodDetector_INCLUDE_PATH)
    
    # Find libraries and library paths
    
    set(NodDetector_LIBRARYNAME "noddetector")
    find_library(NodDetector_LIBRARY NAMES ${NodDetector_LIBRARYNAME}
        HINTS ${NodDetector_BASE_INC_PATH}/lib)
     
    if (NOT NodDetector_LIBRARY)
        set(NodDetector_FOUND FALSE)
        set(NodDetector_ERROR_REASON
            "${NodDetector_ERROR_REASON} Library ${NodDetector_LIBRARYNAME} not found.")
    else(NOT NodDetector_LIBRARY)
        get_filename_component(NodDetector_LIBRARY_PATH
            ${NodDetector_LIBRARY} PATH CACHE)
        list(APPEND NodDetector_LIBRARIES ${NodDetector_LIBRARYNAME})
        get_filename_component(NodDetector_BASE_LIB_PATH
            ${NodDetector_LIBRARY_PATH} PATH)
    endif (NOT NodDetector_LIBRARY)
    
    if (NodDetector_FOUND)
        message(STATUS "NodDetector libraries ${NodDetector_LIBRARIES}")
        message(STATUS "NodDetector library path ${NodDetector_LIBRARY_PATH}")
    endif (NodDetector_FOUND)
    
    if (NOT NodDetector_FOUND)
        if(NodDetector_FIND_REQUIRED)
            message(SEND_ERROR "Unable to find NodDetector.\n${NodDetector_ERROR_REASON}")
        else(NodDetector_FIND_REQUIRED)
            message(STATUS "Unable to find NodDetector: ${NodDetector_ERROR_REASON}")
        endif(NodDetector_FIND_REQUIRED)
    else(NOT NodDetector_FOUND)
        ADD_DEFINITIONS(-D__ND_NOD_DETECTOR_FOUND__)
        link_directories   (${NodDetector_LIBRARY_PATH})
        include_directories(${NodDetector_INCLUDE_PATH})
    endif(NOT NodDetector_FOUND)

else (NOT NONODDETECTOR)
    set(NodDetector_FOUND FALSE)
    message(STATUS "Nod detector disabled manually!")
endif(NOT NONODDETECTOR)
