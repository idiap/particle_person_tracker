# Try to find libopenpose.so
#
# The following variables will be set
# - OpenPose_FOUND
# - OpenPose_INCLUDE_DIRS
# - OpenPose_LIBS

SET(OpenPose_FOUND FALSE)

MESSAGE(STATUS "Try to find OpenPose in <${OpenPose_PATH}>")

IF(NOT "${OpenPose_PATH}" STREQUAL "")

  FIND_PATH(OpenPose_INCLUDE_DIRS openpose/headers.hpp
    HINTS ${OpenPose_PATH}/include)

  FIND_LIBRARY(OpenPose_LIBRARY
    NAMES openpose
    HINTS ${OpenPose_PATH}/lib)

  FIND_LIBRARY(Caffe_LIBRARY
    NAMES caffe
    HINTS ${OpenPose_PATH}/lib)

  SET(OpenPose_LIBS "")
  LIST(APPEND OpenPose_LIBS ${Caffe_LIBRARY})
  LIST(APPEND OpenPose_LIBS ${OpenPose_LIBRARY})

  MESSAGE(STATUS "OpenPose_INCLUDE_DIRS <${OpenPose_INCLUDE_DIRS}>")
  MESSAGE(STATUS "OpenPose_LIBS <${OpenPose_LIBS}>")

  IF(NOT "${OpenPose_LIBS}" STREQUAL "" AND NOT "${OpenPose_INCLUDE_DIRS}" STREQUAL "")
    SET(OpenPose_FOUND TRUE)
  ENDIF()

ENDIF()
