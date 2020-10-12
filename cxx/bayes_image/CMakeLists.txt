# Configures and builds BayesImage module
# Vasil Khalidov - 11 May 2012

project(ttrack-cxx-bayes_image)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "bayes_filter;utils;opencvplus;image_processing") # other TTrack subprojects

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS serialization)

#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#    add_executable(progname file1.cxx file2.cxx)
#    target_link_libraries(progname ${Boost_LIBRARIES})
#endif()

file(GLOB src "src/*.cc")

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${CMotion2D_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
list(APPEND shared ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

include_directories(../render/)

ttrack_library(bayes_image "${src}" "${deps}" "${shared}")

# Tests

# for boost-python?
include_directories(SYSTEM ${python_INCLUDE_DIRS})
link_directories(${python_LIBRARY_DIRS})
list(APPEND shared ${PYTHON_LIBRARY})

#ttrack_test(bayes_image HeadPoseHogModel "test/HeadPoseHogModel.cc" "${shared}")
#ttrack_test(bayes_image HeadPoseSkinModel "test/HeadPoseSkinModel.cc" "${shared}")

#ttrack_test(bayes_image HeadPoseHogModelDiscrimination "test/HeadPoseHogModelDiscrimination.cc" "${shared}")
