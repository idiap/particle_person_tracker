SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g")

SET(LIB_SOURCES
    ../FaceTrackerConfig.cpp ../MainModel.cc ../LongTermTrackerManager.cc
    ../MainWindow.cc ../TrackerManager.cc
    ../LongTermTrackerManager.cc
    ../ModelDumper_TA2Format.cc ../ModelDumper_MOTFormat.cc ../ModelDumper.cc ../ModelDumperFactory.cc
    ../ModelDumper_CSVFormat.cc
    ../NoddingDetector.cc)

if (TTRACK_ROS_INTEGRATION_FOUND)
    list(APPEND LIB_SOURCES ../ModelDumper_ROSFormat2.cc)
endif (TTRACK_ROS_INTEGRATION_FOUND)

# Add modules as dependencies

set(dependencies "utils;image_processing;bayes_image;vfoa;opencvplus")

set(deps_list "")
set(header_list "")
foreach(dep ${dependencies})
    string(TOUPPER "${dep}" DEP)
    if (NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
        list(APPEND deps_list ttrack_${dep})
    endif(NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
    list(APPEND header_list "${TTRACK_${DEP}_HEADER_DIRS}")
endforeach(dep)
list(REMOVE_DUPLICATES header_list)

include_directories(${header_list})
include_directories(..)

include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS} ${RSTSANDBOX_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS} ${NodDetector_LIBRARY_PATH})

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

list(APPEND deps_list
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARY}
  boost_signals
  ${TORCH_LIBRARIES}
  )

if (VfoaModule_FOUND)
    include_directories(${VfoaModule_INCLUDE_PATH})
    link_directories(${VfoaModule_LIBRARY_PATH})
    list(APPEND deps_list ${VfoaModule_LIBRARIES})
endif (VfoaModule_FOUND)

if (NodDetector_FOUND)
    include_directories(${NodDetector_INCLUDE_PATH})
    link_directories(${NodDetector_LIBRARY_PATH})
    list(APPEND deps_list ${NodDetector_LIBRARIES})
endif (NodDetector_FOUND)


ADD_LIBRARY(trackermodel SHARED ${LIB_SOURCES})
TARGET_LINK_LIBRARIES(trackermodel ${deps_list})

ADD_EXECUTABLE(openpose_facetrack OpenPoseTracker.cpp)
TARGET_LINK_LIBRARIES(openpose_facetrack trackermodel)
INSTALL(TARGETS openpose_facetrack RUNTIME DESTINATION bin)


SET(CONF_FILES haarcascade_frontalface_alt.xml
               haarcascade_profilefaceL.xml
               haarcascade_profilefaceR.xml
               facedetectorstats_frontal.data
               facedetectorstats_left.data
               facedetectorstats_right.data
               noddetector.data
               hogmodel.xml
               skinmodel.xml
               hogmodel_BBcorrected.xml
               skinmodel_BBcorrected.xml
               color_prior_hair_continuous_64bit.data
               color_prior_skin_continuous_64bit.data
               color_prior_background_discrete_4_64bit.data
               color_prior_clothes_discrete_4_64bit.data
               color_prior_hair_discrete_4_64bit.data
               color_prior_skin_discrete_4_64bit.data
               color_prior_background_discrete_8_64bit.data
               color_prior_clothes_discrete_8_64bit.data
               color_prior_hair_discrete_8_64bit.data
               color_prior_skin_discrete_8_64bit.data
               color_prior_background_discrete_16_64bit.data
               color_prior_clothes_discrete_16_64bit.data
               color_prior_hair_discrete_16_64bit.data
               color_prior_skin_discrete_16_64bit.data
               pim_back_prior_head_128.data
               pim_front_prior_head_128.data
               pim_profileL_prior_head_128.data
               pim_profileR_prior_head_128.data
               pim_general_prior_head_128.data
               pim_back_prior_UB_128.data
               pim_front_prior_UB_128.data
               pim_profileL_prior_UB_128.data
               pim_profileR_prior_UB_128.data
               pim_general_prior_UB_128.data
               pim_back_prior_UBB_128.data
               pim_front_prior_UBB_128.data
               pim_profileL_prior_UBB_128.data
               pim_profileR_prior_UBB_128.data
               pim_general_prior_UBB_128.data
               pim_front_prior_Carl_128.data
               pim_back_prior_head_32.data
               pim_front_prior_head_32.data
               pim_profileL_prior_head_32.data
               pim_profileR_prior_head_32.data
               pim_general_prior_head_32.data
               pim_back_prior_UB_32.data
               pim_front_prior_UB_32.data
               pim_profileL_prior_UB_32.data
               pim_profileR_prior_UB_32.data
               pim_general_prior_UB_32.data
               pim_back_prior_UBB_32.data
               pim_front_prior_UBB_32.data
               pim_profileL_prior_UBB_32.data
               pim_profileR_prior_UBB_32.data
               pim_general_prior_UBB_32.data
               pim_front_prior_Carl_32.data
               idiap_nao_calib_640x480.yml
               idiap_nao_calib_320x240.yml
               biu_nao_calib_640x480.yml
               biu_nao_calib_320x240.yml
               inria_nao_calib_640x480.yml)

message(STATUS "> configure files source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "> copy configure files for executable ${CURRENT_NAME}: ${CONF_FILES}")
message(STATUS "> configure files destination directory: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
FOREACH(CONF_FILE ${CONF_FILES})
   configure_file(
     "${PROJECT_SOURCE_DIR}/../config_resources/${CONF_FILE}"
       "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config_resources/${CONF_FILE}" COPYONLY)
   INSTALL(FILES "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config_resources/${CONF_FILE}" DESTINATION share/VFOA/config)
ENDFOREACH(CONF_FILE ${CONF_FILES})


SET(CONF_SETUP_FILES config_cv.ini config_openpose.ini)

FOREACH(CONF_FILE ${CONF_SETUP_FILES})
    GET_FILENAME_COMPONENT(ABS_RUNTIME_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ABSOLUTE)
    SET(VFOA_CONFIG_FOLDER "${ABS_RUNTIME_PATH}/config_resources")
    # SET(CPM_MODEL_FOLDER "$ENV{HOME}/models/cpm")
    SET(OPENPOSE_MODEL_FOLDER "${OpenPose_PATH}/../../models/")
    SET(OPENHEADPOSE_MODEL_FOLDER "${OpenHeadPose_PATH}/../../models/")
    configure_file(
      "${PROJECT_SOURCE_DIR}/../config_resources/${CONF_FILE}"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CONF_FILE}" @ONLY)

    GET_FILENAME_COMPONENT(ABS_INSTALL_PATH ${CMAKE_INSTALL_PREFIX} ABSOLUTE)
    SET(VFOA_CONFIG_FOLDER "${ABS_INSTALL_PATH}/share/VFOA/config")
    configure_file(
      "${PROJECT_SOURCE_DIR}/../config_resources/${CONF_FILE}"
        "${CMAKE_CURRENT_BINARY_DIR}/${CONF_FILE}" @ONLY)
    INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CONF_FILE}" DESTINATION share/VFOA/config)
ENDFOREACH(CONF_FILE ${CONF_SETUP_FILES})


if (TTRACK_ROS_INTEGRATION_FOUND)
    list(APPEND SOURCES ../ModelDumper_ROSFormat2.cc)
    list(APPEND SOURCES_CPM ../ModelDumper_ROSFormat2.cc)
endif (TTRACK_ROS_INTEGRATION_FOUND)
