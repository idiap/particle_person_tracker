# Configures a remote ttrack installation
# Vasil Khalidov - 09 May 2010

# This file is installed on every TTrack build so *external* code can compile in
# a few easy steps. If you want to change the way TTrack itself is compiled,
# this is *not* the place.

# Locates and loads all TTrack exported dependencies
find_file(ttrack_BUILD_INFO ttrack.cmake)
include(${ttrack_BUILD_INFO})

# Defines the includes
get_filename_component(ttrack_CMAKE_DIR ${ttrack_BUILD_INFO} PATH)
get_filename_component(ttrack_SHARE_DIR ${ttrack_CMAKE_DIR} PATH)
get_filename_component(ttrack_PREFIX ${ttrack_SHARE_DIR} PATH)

# Loads all externals
find_file(ttrack_DEPENDENCIES_FILE ttrack-external.cmake)
include("${ttrack_DEPENDENCIES_FILE}")

set(ttrack_INCLUDE_DIRS ${ttrack_PREFIX}/include/ttrack)
set(ttrack_LIBRARY_DIRS ${ttrack_PREFIX}/lib)
foreach(dep ${ttrack_DEPENDENCIES})
  find_package(${dep})
  set(ttrack_INCLUDE_DIRS "${ttrack_INCLUDE_DIRS};${${dep}_INCLUDE_DIRS}")
  set(ttrack_LIBRARY_DIRS "${ttrack_LIBRARY_DIRS};${${dep}_LIBRARY_DIRS}")
endforeach(dep ${ttrack_DEPENDENCIES})
list(REMOVE_DUPLICATES ttrack_INCLUDE_DIRS)
list(REMOVE_DUPLICATES ttrack_LIBRARY_DIRS)

# This macro helps users to build TTrack-based executables
macro(ttrack_add_executable name sources dependencies)
  include_directories(${ttrack_INCLUDE_DIRS})
  link_directories(${ttrack_LIBRARY_DIRS})
  add_executable(${name} ${sources})
  foreach(dep ${dependencies})
    target_link_libraries(${name} ttrack_${dep})
  endforeach(dep ${dependencies})
endmacro(ttrack_add_executable name sources dependencies)
