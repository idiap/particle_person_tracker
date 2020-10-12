# Configures and builds ImageProcessing module
# Vasil Khalidov - 10 May 2012

project(ttrack-cxx-image_processing)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "bayes_filter;utils;opencvplus") # other TTrack subprojects

file(GLOB src "src/*.cc")

if (NOT TTRACK_RSB_INTEGRATION_FOUND)
    list(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/ip_RsbImageProvider.cc)
else(NOT TTRACK_RSB_INTEGRATION_FOUND)
    list(APPEND shared ${TTRACK_RSB_INTEGRATION_LIBS})
endif(NOT TTRACK_RSB_INTEGRATION_FOUND)

if (NOT TTRACK_ROS_INTEGRATION_FOUND)
    list(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/ip_RosImageProvider.cc)
else(NOT TTRACK_ROS_INTEGRATION_FOUND)
  list(APPEND shared ${catkin_LIBRARIES})
  # include_directories(${catkin_INCLUDE_DIRS})
endif(NOT TTRACK_ROS_INTEGRATION_FOUND)

if (NOT StereoMatch_FOUND)
    list(REMOVE_ITEM src ${CMAKE_CURRENT_SOURCE_DIR}/src/ip_DisparityImageProcessor.cc)
else(NOT StereoMatch_FOUND)
    list(APPEND shared ${StereoMatch_LIBRARIES} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY})
endif(NOT StereoMatch_FOUND)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMotion2D_INCLUDE_PATH})
link_directories(${OpenCV_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${CMotion2D_LIBRARY_PATH})
list(APPEND shared ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CMotion2D_LIBRARIES})

ttrack_library(image_processing "${src}" "${deps}" "${shared}")

# Tests:

# for boost-python?
include_directories(SYSTEM ${python_INCLUDE_DIRS})
link_directories(${python_LIBRARY_DIRS})
list(APPEND shared ${PYTHON_LIBRARY})

#ttrack_test(image_processing SkinFeatureDistance "test/SkinFeatureDistance.cc" "${shared}")
#ttrack_test(image_processing HogFeatureDistance "test/HogFeatureDistance.cc" "${shared}")
#ttrack_test(image_processing RoiWindow "test/RoiWindow.cc" "${shared}")
#ttrack_test(image_processing TrackingMemoryImage "test/TrackingMemoryImage.cc" "${shared}")
#ttrack_test(image_processing MotionEstimationInria "test/DenseMotionProcessorInria.cc" "${shared}")
#ttrack_test(image_processing SkinColourProcessor "test/SkinColourProcessor.cc" "${shared}")
