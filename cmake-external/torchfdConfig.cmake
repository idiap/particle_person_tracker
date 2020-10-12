# - Find Torch3 face detector
# This module finds if Torch3 face detector is installed and determines where
# the include files and libraries are. This code sets the following variables:
#
#  TorchFaceDetector_FOUND        - have the Torch face detector libs been found
#  TorchFaceDetector_LIBRARIES     - names of the Torch face detector libraries
#  TorchFaceDetector_INCLUDE_PATH  - path to the Torch face detector includes
#  TorchFaceDetector_LIBRARY_PATH  - path to the Torch face detector libraries
#  TorchFaceDetector_RESOURCE_PATH - path to the Torch face detector resources
#

set(TorchFaceDetector_FOUND TRUE)

set(TorchFaceDetector_LIBRARYNAMES release-devel_patternrecognition torch_core
    torch_distributions torch_gradients torch_matrix vision_addons
    vision_illumination torch_convolutions torch_datasets TorchFaceDetector
    torch_kernels torch_nonparametrics vision_core vision_patternrecognition
)

# Find include files and include paths

find_path(TorchFaceDetector_INCLUDE_PATH
    NAMES torch3/TorchFaceDetector.h)

if (NOT TorchFaceDetector_INCLUDE_PATH)
    set(TorchFaceDetector_FOUND FALSE)
    set(TorchFaceDetector_ERROR_REASON
        "${TorchFaceDetector_ERROR_REASON} Includes not found.")
else(NOT TorchFaceDetector_INCLUDE_PATH)
    get_filename_component(TorchFaceDetector_BASE_PATH
        ${TorchFaceDetector_INCLUDE_PATH} PATH)
    message(STATUS "Torch base path ${TorchFaceDetector_BASE_PATH}")
endif (NOT TorchFaceDetector_INCLUDE_PATH)

# Find libraries and library paths

set(TorchFaceDetector_LIBRARIES_FOUND TRUE)
foreach ( LIBRARY ${TorchFaceDetector_LIBRARYNAMES})
    find_library(TorchFaceDetector_${LIBRARY}_LIBRARY
        NAMES ${LIBRARY}
        HINTS ${TorchFaceDetector_BASE_PATH}/lib/torch3)
    if (NOT TorchFaceDetector_${LIBRARY}_LIBRARY)
        set(TorchFaceDetector_FOUND FALSE)
        set(TorchFaceDetector_LIBRARIES_FOUND FALSE)
        set(TorchFaceDetector_MISSING_LIBRARIES
            ${TorchFaceDetector_MISSING_LIBRARIES} ${LIBRARY})
    endif (NOT TorchFaceDetector_${LIBRARY}_LIBRARY)
endforeach ( LIBRARY ${TorchFaceDetector_LIBRARYNAMES})

if (NOT TorchFaceDetector_LIBRARIES_FOUND)
    set(TorchFaceDetector_ERROR_REASON
        "${TorchFaceDetector_ERROR_REASON} Libraries not found:")
    foreach ( LIBRARY ${TorchFaceDetector_MISSING_LIBRARIES})
        set(TorchFaceDetector_ERROR_REASON
            "${TorchFaceDetector_ERROR_REASON} ${LIBRARY}")
    endforeach ( LIBRARY ${TorchFaceDetector_MISSING_LIBRARIES})
    set(TorchFaceDetector_ERROR_REASON
        "${TorchFaceDetector_ERROR_REASON}. ")
endif(NOT TorchFaceDetector_LIBRARIES_FOUND)

# Find resources

find_path(TorchFaceDetector_RESOURCE_PATH
    NAMES torch3/frontal/mct4.cascade
    HINTS ${TorchFaceDetector_BASE_PATH}/share/)

if (NOT TorchFaceDetector_RESOURCE_PATH)
    set(TorchFaceDetector_FOUND FALSE)
    set(TorchFaceDetector_ERROR_REASON
        "${TorchFaceDetector_ERROR_REASON} Resources not found.")
else(NOT TorchFaceDetector_RESOURCE_PATH)
    message(STATUS "Torch face detector resources ${TorchFaceDetector_RESOURCE_PATH}")
endif (NOT TorchFaceDetector_RESOURCE_PATH)

# Report results 

if (NOT TorchFaceDetector_FOUND)
    if(TorchFaceDetector_FIND_REQUIRED)
        message(SEND_ERROR "Unable to find Torch3 face detector.\n${TorchFaceDetector_ERROR_REASON}")
    else(TorchFaceDetector_FIND_REQUIRED)
        message(STATUS "Unable to find Torch3 face detector: ${TorchFaceDetector_ERROR_REASON}")
    endif(TorchFaceDetector_FIND_REQUIRED)
endif(NOT TorchFaceDetector_FOUND)
