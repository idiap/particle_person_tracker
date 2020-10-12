# Configures a remote noddetector installation
# Vasil Khalidov - 04 March 2013

# This file is installed on every NodDetector build so *external* code can
# compile in a few easy steps. If you want to change the way NodDetector
# itself is compiled, this is *not* the place.

#  This code sets the following variables:
#
#  noddetector_INCLUDE_DIRS  - path to the NodDetector includes
#  noddetector_LIBRARY_DIRS  - path to the NodDetector libraries

# Locates and loads all NodDetector exported dependencies
find_file(noddetector_BUILD_INFO noddetector.cmake)
include(${noddetector_BUILD_INFO})

# Defines the includes
get_filename_component(noddetector_CMAKE_DIR ${noddetector_BUILD_INFO} PATH)
get_filename_component(noddetector_SHARE_DIR ${noddetector_CMAKE_DIR} PATH)
get_filename_component(noddetector_PREFIX ${noddetector_SHARE_DIR} PATH)

# Loads all externals
find_file(noddetector_DEPENDENCIES_FILE noddetector-external.cmake)
include("${noddetector_DEPENDENCIES_FILE}")

set(noddetector_INCLUDE_DIRS ${noddetector_PREFIX}/include/noddetector)
set(noddetector_LIBRARY_DIRS ${noddetector_PREFIX}/lib)
foreach(dep ${noddetector_DEPENDENCIES})
  find_package(${dep})
  set(noddetector_INCLUDE_DIRS "${noddetector_INCLUDE_DIRS};${${dep}_INCLUDE_DIRS}")
  set(noddetector_LIBRARY_DIRS "${noddetector_LIBRARY_DIRS};${${dep}_LIBRARY_DIRS}")
endforeach(dep ${noddetector_DEPENDENCIES})

list(REMOVE_DUPLICATES noddetector_INCLUDE_DIRS)
list(REMOVE_DUPLICATES noddetector_LIBRARY_DIRS)
