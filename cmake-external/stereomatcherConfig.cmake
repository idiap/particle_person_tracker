# - Find stereomatcher module
# This module finds if StereoMatchermodule is installed and determines where the
# include files and libraries are. This code sets the following variables:
#
#  StereoMatch_FOUND         - have the StereoMatch libs been found
#  StereoMatch_LIBRARIES     - names of the StereoMatch libraries
#  StereoMatch_INCLUDE_PATH  - path to the StereoMatch includes
#  StereoMatch_LIBRARY_PATH  - path to the StereoMatch libraries
#  StereoMatch_RESOURCE_PATH - path to the StereoMatch resources
#

if (NOT NOSTEREOMATCHER)

    set(StereoMatch_FOUND TRUE)
    
    # Find include files and include paths
    
    find_path(StereoMatch_INCLUDE_PATH
        NAMES stereomatcher/stereomatcher.h stereomatcher/stereoconfiguration.h)
    
    if (NOT StereoMatch_INCLUDE_PATH)
        set(StereoMatch_FOUND FALSE)
        set(StereoMatch_ERROR_REASON
            "${StereoMatch_ERROR_REASON} Includes not found.")
    else(NOT StereoMatch_INCLUDE_PATH)
        get_filename_component(StereoMatch_BASE_INC_PATH
            ${StereoMatch_INCLUDE_PATH} PATH)
        set(StereoMatch_INCLUDE_PATH ${StereoMatch_INCLUDE_PATH}/stereomatcher)
        message(STATUS "StereoMatch includes ${StereoMatch_INCLUDE_PATH}")
    endif (NOT StereoMatch_INCLUDE_PATH)
    
    # Find libraries and library paths
    
    set(StereoMatch_LIBRARYNAME "stereomatch")
    set(StereoMatchRpg_LIBRARYNAME "rpgstereo")
    find_library(StereoMatch_LIBRARY NAMES ${StereoMatch_LIBRARYNAME}
        HINTS ${StereoMatch_BASE_INC_PATH}/lib/stereomatcher)
    find_library(StereoMatchRpg_LIBRARY NAMES ${StereoMatchRpg_LIBRARYNAME}
        HINTS ${StereoMatch_BASE_INC_PATH}/lib/stereomatcher)
     
    if (NOT StereoMatch_LIBRARY)
        set(StereoMatch_FOUND FALSE)
        set(StereoMatch_ERROR_REASON
            "${StereoMatch_ERROR_REASON} Library ${StereoMatch_LIBRARYNAME} not found.")
    else(NOT StereoMatch_LIBRARY)
        get_filename_component(StereoMatch_LIBRARY_PATH
            ${StereoMatch_LIBRARY} PATH CACHE)
        list(APPEND StereoMatch_LIBRARIES ${StereoMatch_LIBRARYNAME})
        get_filename_component(StereoMatch_BASE_LIB_PATH
            ${StereoMatch_LIBRARY_PATH} PATH)
    endif (NOT StereoMatch_LIBRARY)
    
    if (NOT StereoMatchRpg_LIBRARY)
        set(StereoMatch_FOUND FALSE)
        set(StereoMatch_ERROR_REASON
            "${StereoMatch_ERROR_REASON} Library ${StereoMatchRpg_LIBRARYNAME} not found.")
    else(NOT StereoMatchRpg_LIBRARY)
        get_filename_component(StereoMatch_LIBRARY_PATH
            ${StereoMatch_LIBRARY} PATH CACHE)
        list(APPEND StereoMatch_LIBRARIES ${StereoMatchRpg_LIBRARYNAME})
        get_filename_component(StereoMatch_BASE_LIB_PATH
            ${StereoMatch_LIBRARY_PATH} PATH)
    endif (NOT StereoMatchRpg_LIBRARY)
    
    if (StereoMatch_FOUND)
        message(STATUS "StereoMatch libraries ${StereoMatch_LIBRARIES}")
        message(STATUS "StereoMatch library path ${StereoMatch_LIBRARY_PATH}")
    endif (StereoMatch_FOUND)
    
    FIND_PACKAGE(Qt4 QUIET COMPONENTS QtCore QtGui)
    if (NOT QT_FOUND)
        set(StereoMatch_FOUND FALSE)
    else (NOT QT_FOUND)
        INCLUDE(${QT_USE_FILE})
    endif (NOT QT_FOUND)
    
    if (NOT StereoMatch_FOUND)
        if(StereoMatch_FIND_REQUIRED)
            message(SEND_ERROR "Unable to find StereoMatcher.\n${StereoMatch_ERROR_REASON}")
        else(StereoMatch_FIND_REQUIRED)
            message(STATUS "Unable to find StereoMatch: ${StereoMatch_ERROR_REASON}")
        endif(StereoMatch_FIND_REQUIRED)
    else(NOT StereoMatch_FOUND)
        ADD_DEFINITIONS(-D__STEREO_MATCHER_FOUND__)
        link_directories   (${StereoMatch_LIBRARY_PATH})
        include_directories(${StereoMatch_INCLUDE_PATH})
    endif(NOT StereoMatch_FOUND)
else (NOT NOSTEREOMATCHER)
    set(StereoMatch_FOUND FALSE)
    message(STATUS "StereoMatcher disabled manually!")
endif(NOT NOSTEREOMATCHER)
