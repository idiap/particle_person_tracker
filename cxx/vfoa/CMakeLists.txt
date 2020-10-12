# Configures and builds BayesImage module
# Vasil Khalidov - 11 May 2012

project(ttrack-cxx-vfoa)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "bayes_filter;utils;opencvplus;image_processing;bayes_image") # other TTrack subprojects

file(GLOB src "src/*.cc")

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
list(APPEND shared ${OpenCV_LIBRARIES})

ttrack_library(vfoa "${src}" "${deps}" "${shared}")