# Configures and builds C++ TTrack modules
# Vasil Khalidov - 10 May 2012

project(ttrack-cxx)
cmake_minimum_required(VERSION 2.6)

# Search for RSB to enable RSB-dependent components

ttrack_check_rsb()

message("##################################### ${RSB_VERSION}")
if (RSB_VERSION VERSION_LESS 0.9)
    add_definitions(-D__RSBINTEGRATION_OLD__)
endif ()

# Base functionality

add_subdirectory(utils)
add_subdirectory(bayes_filter)

# if (RSC_FOUND AND RSB_FOUND AND PROTOBUF_FOUND AND RST_FOUND)
#     add_definitions(-D__RSBINTEGRATION_FOUND__)
#     add_subdirectory(rsbintegration)
# endif (RSC_FOUND AND RSB_FOUND AND PROTOBUF_FOUND AND RST_FOUND)

add_subdirectory(opencvplus)
add_subdirectory(image_processing)

add_subdirectory(bayes_image)
add_subdirectory(vfoa)

# rendering utilities

add_subdirectory(render)

# Stefan Duffner's code
# TODO: refactor to eliminate these parts

# add_subdirectory(torchlite)
# add_subdirectory(tracker_SD)

# Examples - various executables

add_subdirectory(examples)
