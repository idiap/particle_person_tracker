# Configures and builds TTrack modules
# Vasil Khalidov - 09 May 2012

cmake_minimum_required(VERSION 3.0)

PROJECT(particle_person_tracker)

SET(CMAKE_CXX_FLAGS "-std=c++11")

######################################################################
# OpenPose
######################################################################

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules")

FIND_PACKAGE(Boost 1.46 REQUIRED COMPONENTS system thread filesystem)

SET(external_LIBS "")

FIND_PACKAGE(OpenPose REQUIRED)
IF(OpenPose_FOUND)
  MESSAGE(STATUS "Found OpenPose")
  INCLUDE_DIRECTORIES(${OpenPose_INCLUDE_DIRS})
  ADD_DEFINITIONS(-D__OPENPOSE_FOUND__)
  LIST(APPEND external_LIBS ${OpenPose_LIBS})
#   SET(OPENPOSE_LIBRARIES
#     ${OPENPOSE_LIBRARY}
#     ${PROTOBUF_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     ${Boost_LIBRARIES})
#   MESSAGE("OPENPOSE_LIBRARIES ${OPENPOSE_LIBRARIES}")
#   FIND_PACKAGE(Protobuf REQUIRED)
#   FIND_PACKAGE(CUDA 8.0 REQUIRED)
#   INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIR})
#   INCLUDE_DIRECTORIES(SYSTEM ${CUDA_INCLUDE_DIRS})
#   INCLUDE_DIRECTORIES(SYSTEM ${PROTOBUF_INCLUDE_DIR})
ENDIF()

FIND_PACKAGE(OpenHeadPose)
IF(OpenHeadPose_FOUND)
  MESSAGE(STATUS "Found OpenHeadPose")
  ADD_DEFINITIONS(-D__OPENHEADPOSE_FOUND__)
  INCLUDE_DIRECTORIES(${OpenHeadPose_INCLUDE_DIRS})
  LIST(APPEND external_LIBS ${OpenHeadPose_LIBS})
#   INCLUDE_DIRECTORIES(${OPENHEADPOSE_INCLUDE})
#   SET(OPENHEADPOSE_LIBRARIES
#     ${OPENHEADPOSE_LIBRARY}
#     ${PROTOBUF_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     ${Boost_LIBRARIES})
#   MESSAGE("OPENHEADPOSE_LIBRARIES ${OPENHEADPOSE_LIBRARIES}")
ENDIF()


############ System information

set(TTRACK_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
set(TTRACK_SYSTEM_NAME ${CMAKE_SYSTEM_NAME})
if(APPLE)
  set(TTRACK_SYSTEM_PROCESSOR "x86_64") #forced, see __LP64__ comment bellow
  set(TTRACK_SYSTEM_NAME "macosx")
endif()

string(TOLOWER
    "${TTRACK_SYSTEM_NAME}-${TTRACK_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}"
    TTRACK_PLATFORM_STR)
set(TTRACK_PLATFORM "${TTRACK_PLATFORM_STR}" CACHE STRING
    "The name of the platform TTrack is being built to")

########Disable qt keywords
add_definitions(-DQT_NO_KEYWORDS)

############ TTrack version

set(TTRACK_VERSION_INTERNAL "1.0.0" CACHE INTERNAL
    "The version of TTrack that is currently being built")
if(NOT TTRACK_VERSION)
    set(TTRACK_VERSION ${TTRACK_VERSION_INTERNAL})
endif()

############ Library path option

option(SET_PUBLIC_LIBRARY_PATH
    "Use `ld -rpath' (Linux) or `install_name_tool' (Apple) when linking public
    libraries, executables. This option does not cover python bindings to C++
    installed into Python directories." ON)

############ Language options

# NOTE: one cannot build Python bindings without C++ libraries. However,
# Python bindings can be built separately, provided C++ libraries were built
# previously
option(TTRACK_BUILD_CXX "Builds C++ infrastructure." ON)
# option(TTRACK_BUILD_PYTHON "Builds Python bindings." OFF)

############ Compilation flags

message(STATUS "TTrack version '${TTRACK_VERSION}'")

# Force __LP64__ scheme on Mac OSX
if(APPLE)
    set(COMMON_FLAGS "-m64")
    set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-isystem ")
    set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")
endif(APPLE)

set(COMMON_FLAGS "${COMMON_FLAGS} -fopenmp -pthread -pedantic -Wno-variadic-macros -Wno-long-long -Wall")

if (CMAKE_COMPILER_IS_GNUCC)
#  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mtune=generic -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -DTTRACK_DEBUG")
else (CMAKE_COMPILER_IS_GNUCC)
  # Then, it must be clang/clang++
  set(COMMON_FLAGS "${COMMON_FLAGS} -Qunused-arguments")
#  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mtune=generic -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -DTTRACK_DEBUG")
endif (CMAKE_COMPILER_IS_GNUCC)

set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} CACHE STRING
    "Flags used by the compiler during release builds" FORCE)
set(CMAKE_C_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} CACHE STRING
    "Flags used by the compiler during release builds" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG} CACHE STRING
    "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_C_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG} CACHE STRING
    "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${CMAKE_CXX_FLAGS}" CACHE STRING
    "Flags used by the compiler during all build types." FORCE)
set(CMAKE_C_FLAGS "${COMMON_FLAGS} ${CMAKE_C_FLAGS}" CACHE STRING
    "Flags used by the compiler during all build types." FORCE)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" )
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs")

if(SET_PUBLIC_LIBRARY_PATH)
    if (APPLE)
        set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    else()
        # This is our RPATH policy under Linux
        set(CMAKE_SKIP_BUILD_RPATH  FALSE)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    endif()
endif()

# This will make sure we consider pkg-config directories that may have been
# added when the user set CMAKE_PREFIX_PATH.
if(CMAKE_PREFIX_PATH)
    string(REPLACE ":" ";" PKG_CONFIG_CHECK_PATH ${CMAKE_PREFIX_PATH})
    foreach (ext ${PKG_CONFIG_CHECK_PATH})
        if(EXISTS ${ext}/lib/pkgconfig)
            set(ENV{PKG_CONFIG_PATH}
                "${ext}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
        endif()
    endforeach()
endif()

############ External dependencies

# Find catkin macros and libraries, also other catkin packages.

find_package(catkin
  OPTIONAL_COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  cmake_modules
  image_transport
  perception_msgs
  tf
  tf2
  tf2_ros
)

if(catkin_FOUND)
  # ADD_MESSAGE_FILES(FILES
  #   BoundingBox.msg
  #   ClassificationResult.msg
  #   ClassWithProbability.msg
  #   Face.msg
  #   HeadObject.msg
  #   HeadObjects.msg
  #   PersonId.msg
  #   Vec3DFloat.msg)
    # GENERATE_MESSAGES(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
    CATKIN_PACKAGE(CATKIN_DEPENDS roscpp image_transport perception_msgs)
    # CATKIN_PACKAGE(CATKIN_DEPENDS message_runtime roscpp image_transport)
    ADD_DEFINITIONS(-D__ROSINTEGRATION_FOUND__)
    SET(TTRACK_ROS_INTEGRATION_FOUND TRUE CACHE INTERNAL "ROS integration found flag")
    # INCLUDE_DIRECTORIES(SYSTEM ${catkin_INCLUDE_DIRS})
    # MESSAGE("############################################################")
    # MESSAGE(WARNING "catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS}")
    # MESSAGE("############################################################")
    # # FIXME: opencv3 package are a problem to compile
    # SET(to_remove "")
    # FOREACH(item ${catkin_INCLUDE_DIRS})
    #   STRING(REGEX MATCH ".+opencv.+" item ${item})
    #   MESSAGE(WARNING "Got ${item}")
    #   IF(item)
    #     MESSAGE(WARNING "Removing ${item}")
    #     LIST(APPEND to_remove ${item})
    #   ELSE()
    #   ENDIF()
    # ENDFOREACH()

    # FOREACH(item ${to_remove})
    #   MESSAGE(WARNING "Removing ${item}")
    #   LIST(REMOVE_ITEM catkin_INCLUDE_DIRS ${item})
    # ENDFOREACH()

    # MESSAGE(WARNING "catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS}")
    INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
    # MESSAGE("############################################################")
    # MESSAGE("catkin_INCLUDE_DIRS ${catkin_LIBRARIES}")
    # MESSAGE("############################################################")
else(catkin_FOUND)
  MESSAGE(STATUS "Catking was not found")
  SET(CATKIN_PACKAGE_LIB_DESTINATION "/tmp")
endif(catkin_FOUND)


add_subdirectory(3rdparty/cmotion2D)
add_definitions(-D__CMOTION2D_FOUND__)
get_filename_component(CMotion2D_INCLUDE_PATH "3rdparty/cmotion2D" ABSOLUTE)
include_directories(${CMotion2D_INCLUDE_PATH})
get_filename_component(CMotion2D_LIBRARY_PATH "${CATKIN_PACKAGE_LIB_DESTINATION}" ABSOLUTE)
set(CMotion2D_LIBRARY "${CMotion2D_LIBRARY_PATH}/libcmotion2d.so")
link_directories(${CMotion2D_LIBRARY_PATH})

add_subdirectory(3rdparty/NodDetector)
ADD_DEFINITIONS(-D__ND_NOD_DETECTOR_FOUND__)
get_filename_component(NodDetector_INCLUDE_PATH "3rdparty/NodDetector" ABSOLUTE)
include_directories(${NodDetector_INCLUDE_PATH})
get_filename_component(NodDetector_LIBRARY_PATH "${CATKIN_PACKAGE_LIB_DESTINATION}" ABSOLUTE)
set(NodDetector_LIBRARY "${NodDetector_LIBRARY_PATH}/libnoddetector.so")
link_directories(${NodDetector_LIBRARY_PATH})

add_subdirectory(3rdparty/vfoa)
ADD_DEFINITIONS(-D__VFOA_MODULE_FOUND__)
get_filename_component(VfoaModule_INCLUDE_PATH "3rdparty/vfoa/include" ABSOLUTE)
include_directories(${VfoaModule_INCLUDE_PATH})
get_filename_component(VfoaModule_LIBRARY_PATH "${CATKIN_PACKAGE_LIB_DESTINATION}" ABSOLUTE)
set(VfoaModule_LIBRARY "${VfoaModule_LIBRARY_PATH}/libvfoa_module.so")
link_directories(${VfoaModule_LIBRARY_PATH})


include(FindPackageMessage)

# Sets up external dependencies
# Defines TTRACK_DEPENDENCIES and TTRACK_OPTIONALS variables
include(cmake/ttrack-external.cmake)

foreach (ext ${TTRACK_DEPENDENCIES})
  find_package(${ext} REQUIRED PATHS cmake-external)
endforeach ()

foreach (ext ${TTRACK_OPTIONALS})
  find_package(${ext} PATHS cmake-external)
endforeach ()

# This is where we install cmake files for external projects
#set(cmakedir share/cmake/ttrack)
#install(DIRECTORY cmake-external/ DESTINATION ${cmakedir}
#    PATTERN ".svn" EXCLUDE
#    PATTERN "*.cmake")
#install (FILES cmake/ttrack-external.cmake DESTINATION ${cmakedir})

############ Basic build instructions

# Enables the testing framework
enable_testing()

include(cmake/macros.cmake)

set(BUILD_SHARED_LIBS "ON" CACHE BOOL "Build shared libs")

if (TTRACK_BUILD_CXX)
    add_subdirectory(cxx)
endif ()

# if (TTRACK_BUILD_PYTHON)
#     add_subdirectory(python)
# endif ()

#install(EXPORT ttrack DESTINATION ${cmakedir})
#install(FILES cmake/ttrackConfig.cmake DESTINATION ${cmakedir})

# Adds Doxygen and Sphinx generation
include(cmake/documentation.cmake)
