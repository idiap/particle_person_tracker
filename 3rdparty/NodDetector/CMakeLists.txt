# Configures and builds NodDetector project
# Vasil Khalidov - 04 March 2013

project(NodDetector)
cmake_minimum_required(VERSION 2.6)

############ NodDetector version

set(NODDETECTOR_VERSION_INTERNAL "1.0.0" CACHE INTERNAL
    "The version of NodDetector that is currently being built")
if(NOT NODDETECTOR_VERSION)
    set(NODDETECTOR_VERSION ${NODDETECTOR_VERSION_INTERNAL})
endif()

############ Compilation flags

message(STATUS "NodDetector version '${NODDETECTOR_VERSION}'")

set(COMMON_FLAGS "${COMMON_FLAGS} -fopenmp -pthread -pedantic -Wno-variadic-macros -Wno-long-long -Wall")

if (CMAKE_COMPILER_IS_GNUCC)
#  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mtune=generic -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=native -mssse3 -msse2 -msse4 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")
else (CMAKE_COMPILER_IS_GNUCC)
  # Then, it must be clang/clang++
  set(COMMON_FLAGS "${COMMON_FLAGS} -Qunused-arguments")
#  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -ffast-math -mtune=generic -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")
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

include(FindPackageMessage)

# Sets up external dependencies
# Defines NODDETECTOR_DEPENDENCIES and NODDETECTOR_OPTIONALS variables
include(cmake/noddetector-external.cmake)

foreach (ext ${NODDETECTOR_DEPENDENCIES})
  find_package(${ext} REQUIRED PATHS cmake-external)
endforeach ()

foreach (ext ${NODDETECTOR_OPTIONALS})
  find_package(${ext} PATHS cmake-external)
endforeach ()

# This is where we install cmake files for external projects
set(cmakedir share/cmake/noddetector)
install(DIRECTORY cmake-external/ DESTINATION ${cmakedir}
    PATTERN ".svn" EXCLUDE
    PATTERN "*.cmake")
install (FILES cmake/noddetector-external.cmake DESTINATION ${cmakedir})

############ Basic build instructions

set(BUILD_SHARED_LIBS "ON" CACHE BOOL "Build shared libs")

file(GLOB src "src/*.cc")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(noddetector ${src})
target_link_libraries(noddetector ${FFTW_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${GSL_LIBRARIES})
#set_target_properties(noddetector PROPERTIES LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)
if (TTRACK_SOVERSION)
    # adds versioning information
    set_target_properties(noddetector PROPERTIES VERSION ${NODDETECTOR_VERSION})
    set_target_properties(noddetector PROPERTIES SOVERSION ${NODDETECTOR_VERSION})
endif()

install(DIRECTORY noddetector DESTINATION include
    PATTERN ".svn" EXCLUDE
    PATTERN "*.h")

install(TARGETS noddetector EXPORT noddetector
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(EXPORT noddetector DESTINATION ${cmakedir})
#install(FILES cmake/noddetectorConfig.cmake DESTINATION ${cmakedir})

############ Tests

enable_testing()

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/test_reports/")

macro(noddetector_test name src shared)
  set(testname ndtest_${name})
    # Please note we don't install test executables
  add_executable(${testname} ${src})
  target_link_libraries(${testname} noddetector ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY_RELEASE} ${shared})
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/test_reports/")
  add_test(ndtest-${name} "${CMAKE_BINARY_DIR}/bin/${testname}" --log_level=all --output_format=XML)
  set_property(TEST ndtest-${name} APPEND PROPERTY ENVIRONMENT "ND_TESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  set_property(TEST ndtest-${name} APPEND PROPERTY ENVIRONMENT "ND_TESTREPORT_FILE=${CMAKE_BINARY_DIR}/test_reports/${testname}_report.xml")
endmacro()

list(APPEND shared ${Boost_LIBRARIES})

noddetector_test(Load "test/NodDetectorLoad.cc" "${shared}")
noddetector_test(Evaluate "test/NodDetectorEvaluate.cc" "${shared}")
noddetector_test(Detect "test/NodDetectorDetections.cc" "${shared}")
noddetector_test(Utils "test/NodDetectorUtils.cc" "${shared}")
