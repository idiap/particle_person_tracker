# - Find stereomatcher module
# This module checks if VfoaModule is installed and determines where the
# include files and libraries are. This code sets the following variables:
#
#  VfoaModule_FOUND         - have the VfoaModule libs been found
#  VfoaModule_LIBRARIES     - names of the VfoaModule libraries
#  VfoaModule_INCLUDE_PATH  - path to the VfoaModule includes
#  VfoaModule_LIBRARY_PATH  - path to the VfoaModule libraries
#  VfoaModule_RESOURCE_PATH - path to the VfoaModule resources
#

if (NOT NOVFOA)

    set(VfoaModule_FOUND TRUE)
    
    # Find include files and include paths
    
    find_path(VfoaModule_INCLUDE_PATH
        NAMES vfoa_module/vfoa_manager.h)
    
    if (NOT VfoaModule_INCLUDE_PATH)
        set(VfoaModule_FOUND FALSE)
        set(VfoaModule_ERROR_REASON
            "${VfoaModule_ERROR_REASON} Includes not found.")
    else(NOT VfoaModule_INCLUDE_PATH)
        get_filename_component(VfoaModule_BASE_INC_PATH
            ${VfoaModule_INCLUDE_PATH} PATH)
        message(STATUS "VfoaModule includes ${VfoaModule_INCLUDE_PATH}")
    endif (NOT VfoaModule_INCLUDE_PATH)
    
    # Find libraries and library paths
    
    set(VfoaModule_LIBRARYNAME "vfoa_module")
    find_library(VfoaModule_LIBRARY NAMES ${VfoaModule_LIBRARYNAME}
        HINTS ${VfoaModule_BASE_INC_PATH}/lib)
     
    if (NOT VfoaModule_LIBRARY)
        set(VfoaModule_FOUND FALSE)
        set(VfoaModule_ERROR_REASON
            "${VfoaModule_ERROR_REASON} Library ${VfoaModule_LIBRARYNAME} not found.")
    else(NOT VfoaModule_LIBRARY)
        get_filename_component(VfoaModule_LIBRARY_PATH
            ${VfoaModule_LIBRARY} PATH CACHE)
        list(APPEND VfoaModule_LIBRARIES ${VfoaModule_LIBRARYNAME})
        get_filename_component(VfoaModule_BASE_LIB_PATH
            ${VfoaModule_LIBRARY_PATH} PATH)
    endif (NOT VfoaModule_LIBRARY)
    
    if (VfoaModule_FOUND)
        message(STATUS "VfoaModule libraries ${VfoaModule_LIBRARIES}")
        message(STATUS "VfoaModule library path ${VfoaModule_LIBRARY_PATH}")
    endif (VfoaModule_FOUND)
    
    if (NOT VfoaModule_FOUND)
        if(VfoaModule_FIND_REQUIRED)
            message(SEND_ERROR "Unable to find VfoaModule.\n${VfoaModule_ERROR_REASON}")
        else(VfoaModule_FIND_REQUIRED)
            message(STATUS "Unable to find VfoaModule: ${VfoaModule_ERROR_REASON}")
        endif(VfoaModule_FIND_REQUIRED)
    else(NOT VfoaModule_FOUND)
        ADD_DEFINITIONS(-D__VFOA_MODULE_FOUND__)
        link_directories   (${VfoaModule_LIBRARY_PATH})
        include_directories(${VfoaModule_INCLUDE_PATH})
    endif(NOT VfoaModule_FOUND)

else (NOT NOVFOA)
    set(VfoaModule_FOUND FALSE)
    message(STATUS "VFOA module disabled manually!")
endif(NOT NOVFOA)
