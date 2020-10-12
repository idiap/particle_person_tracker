MESSAGE(STATUS "Trying OpenvCV 3")
FIND_PACKAGE(OpenCV 3)
if(NOT OpenCV_FOUND)
  MESSAGE("OpenvCV 3 not found. Trying default import.")
  FIND_PACKAGE(OpenCV)
endif()

find_package_message(OpenCV "Found OpenCV ${OpenCV_VERSION}: ${OpenCV_INCLUDE_DIRS} - ${OpenCV_LIBRARIES}" "[${OpenCV_LIBRARIES}][${OpenCV_INCLUDE_DIRS}]")

include(FindPkgConfig)

# if(CMAKE_VERSION VERSION_LESS "3.0.0")
#     pkg_check_modules(OpenCV opencv)
# else()
#     # starting at cmake-2.8.2, the QUIET option can be used
#     pkg_check_modules(OpenCV QUIET opencv)
# endif()

# MESSAGE("------------------------------------------------------------")
# MESSAGE("CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}")
# MESSAGE("catkin ${catkin_LIBRARIES}")
# MESSAGE("------------------------------------------------------------")


# if(OpenCV_FOUND)
#     MESSAGE("Found OpenCV ${OpenCV_VERSION}")
#     if( ${OpenCV_VERSION} VERSION_LESS "3.0.0")
#       message(FATAL_ERROR "OpenCV version less than minimum version (2.0.0)")
#     elseif(${OpenCV_VERSION} VERSION_GREATER "3.0.0" OR ${OpenCV_VERSION} VERSION_EQUAL "3.0.0")
#         # KLUDGE: remove any dependency on the NOPENCV flag,
#         #         use OPENCV_VERSION instead
#         add_definitions(-DNOPENCV)
#     endif(${OpenCV_VERSION} VERSION_LESS "3.0.0")

#     #checks to see if libcvaux is installed - optional in some systems
#     find_file(OpenCV_CVAUX_H_FOUND NAMES cvaux.h PATHS ${OpenCV_INCLUDE_DIRS})
#     if (NOT OpenCV_CVAUX_H_FOUND)
#         message(WARNING "OpenCV (version ${OpenCV_VERSION}) was found, but cvaux.h was not!")
#     endif()
#     #cannot search for libraries as names may vary in different installations

#     #checks to see if libhighgui is installed - optional in some systems
#     find_file(OpenCV_HIGHGUI_H_FOUND NAMES highgui.h PATHS ${OpenCV_INCLUDE_DIRS})
#     if (NOT OpenCV_HIGHGUI_H_FOUND)
#         message(WARNING "OpenCV (version ${OpenCV_VERSION}) was found, but highgui.h was not!")
#     endif()
#     #cannot search for libraries as names may vary in different installations

#     add_definitions("-DHAVE_OPENCV=1")
#     add_definitions("-DOPENCV_VERSION=\"${OpenCV_VERSION}\"")
#     find_package_message(OpenCV "Found OpenCV ${OpenCV_VERSION}: ${OpenCV_INCLUDE_DIRS} - ${OpenCV_LIBRARIES}" "[${OpenCV_LIBRARIES}][${OpenCV_INCLUDE_DIRS}]")
# endif()
