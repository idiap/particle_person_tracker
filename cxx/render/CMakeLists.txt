# Configures and builds Render module
# Vasil Khalidov - 19 June 2012

project(ttrack-cxx-render)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "image_processing;utils;opencvplus") # other TTrack subprojects

file(GLOB src "src/*.cc")

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
list(APPEND shared ${OpenCV_LIBRARIES})

ttrack_library(render "${src}" "${deps}" "${shared}")