# Try to find libopenpose.so
#
# The following variables will be set
# - OpenPose_FOUND
# - OpenPose_INCLUDE_DIRS
# - OpenPose_LIBS

SET(OpenHeadPose_FOUND FALSE)

MESSAGE(STATUS "Try to find OpenHeadPose in <${OpenHeadPose_PATH}>")

IF(NOT "${OpenHeadPose_PATH}" STREQUAL "")

  FIND_PACKAGE(CUDA REQUIRED)

  FIND_PATH(OpenHeadPose_INCLUDE_DIRS openheadpose/Config.h
    HINTS ${OpenHeadPose_PATH}/include)

  LIST(APPEND OpenHeadPose_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})

  FIND_LIBRARY(OpenHeadPose_LIBRARY
    NAMES openheadpose
    HINTS ${OpenHeadPose_PATH}/lib)

  SET(OpenHeadPose_LIBS "")
  LIST(APPEND OpenHeadPose_LIBS ${OpenHeadPose_LIBRARY})

  MESSAGE(STATUS "OpenHeadPose_INCLUDE_DIRS <${OpenHeadPose_INCLUDE_DIRS}>")
  MESSAGE(STATUS "OpenHeadPose_LIBS <${OpenHeadPose_LIBS}>")

  IF(NOT "${OpenHeadPose_LIBS}" STREQUAL "" AND
     NOT "${OpenHeadPose_INCLUDE_DIRS}" STREQUAL "")
    SET(OpenHeadPose_FOUND TRUE)
  ENDIF()

ENDIF()
