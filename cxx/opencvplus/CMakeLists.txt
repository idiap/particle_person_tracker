# Configures and builds OpenCVPlus module
# Vasil Khalidov - 10 May 2012

project(ttrack-cxx-opencvplus)
cmake_minimum_required(VERSION 3.0)

# Dependencies of this package

set(deps "bayes_filter;utils")  # other TTrack subprojects

file(GLOB src "src/*.cc")
list(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/IdiapCVOpticalFlow.cc)

IF(NOT OpenPose_FOUND)
  LIST(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/cvp_FaceDetectorOpenPose.cc)
ENDIF()

IF(NOT OpenHeadPose_FOUND)
  LIST(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/cvp_FaceDetectorOpenHeadPose.cc)
ENDIF()

IF(NOT OpenPose_FOUND AND NOT OpenHeadPose_FOUND)
  LIST(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/cvp_OpenPoseUtils.cc)
ENDIF()


INCLUDE_DIRECTORIES(SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
LIST(APPEND shared ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${external_LIBS})

ttrack_library(opencvplus "${src}" "${deps}" "${shared}")

# Tests:

# for boost-python?
include_directories(SYSTEM ${python_INCLUDE_DIRS})
link_directories(${python_LIBRARY_DIRS})
list(APPEND shared ${PYTHON_LIBRARY})

ttrack_test(opencvplus PimFeature "test/PimFeature.cc" "${shared}")
ttrack_test(opencvplus Palette "test/Palette.cc" "${shared}")
ttrack_test(opencvplus PaletteBundle "test/PaletteBundle.cc" "${shared}")
ttrack_test(opencvplus FaceColourModel "test/FaceColourModel.cc" "${shared}")
ttrack_test(opencvplus FaceDetectorStatistics "test/FaceDetectorStatistics.cc" "${shared}")
