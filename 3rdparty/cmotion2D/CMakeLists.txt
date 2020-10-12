# Configures and builds CMotion2D module
# Vasil Khalidov - 11 May 2012

set(package "cmotion2d")

project(${package})
cmake_minimum_required(VERSION 2.6)

file(GLOB src "src/[^.]*/*.c*")

set(COMMON_FLAGS "${COMMON_FLAGS} -fopenmp -pthread -pedantic -Wno-variadic-macros -Wno-long-long -Wall -fPIC -D__NO_IMAGEIO_MPEG_ -D__NO_IMAGEIO_PNG_")

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -ffast-math -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -DTTRACK_DEBUG")

set_source_files_properties(${src} PROPERTIES LANGUAGE CXX)
set(INCLUDE_DIRS src/compense src/cprim src/estimation src/image
    src/inc src/interface src/mat_sym src/memoire src/pyramide .)
include_directories(${INCLUDE_DIRS})

add_library(${package} SHARED ${src})
#set_target_properties(${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)

install(TARGETS ${package} EXPORT cmotion2d
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

set(headers "${package}")

install(DIRECTORY ${headers} DESTINATION include
    PATTERN ".svn" EXCLUDE
    PATTERN "*.h")
