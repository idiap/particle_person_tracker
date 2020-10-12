// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainModel - main model for the face tracking application
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MAINMODEL_H__
#define __MAINMODEL_H__

// SYSTEM INCLUDES
#include <list>
#include <string>

#ifdef __VFOA_MODULE_FOUND__
#include <vfoa_manager.h>                    // VFOA manager
#endif

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>                 // ROI
#include <image_processing/ip_Dense2DMotionProcessor.h>    // dense 2D motion processor
#include <image_processing/ip_HogFeatureProducerRounded.h> // HoG feature producer
#include <image_processing/ip_SkinFeatureProducer.h>       // skin feature producer
#include <image_processing/ip_ColourSkinFeatureProducer.h> // colskin feature producer
#include <image_processing/ip_SkinColourProcessor.h>       // skin mask provider
#include <opencvplus/cvp_FaceDetectorStatisticsStorage.h>
#include <opencvplus/cvp_FaceDetectorGroup.h>

#include <bayes_image/bicv_ARHeadPoseDynamicModel.h>     // dynamic model
#include <bayes_image/bicv_HeadPoseLikelihoodModel.h>    // likelihood model
#include <bayes_image/bicv_SkinHogObservationProvider.h> // observation provider
#include <bayes_image/bicv_HeadPoseTracker.h>            // tracker

#include <vfoa/vfoa_CognitiveVfoaModel.h>                // VFOA model

#include <opencvplus/FaceColorModel.h>                   // face color model

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                    // config structure
#include "TrackerManager.h"                         // tracker manager
#include "NoddingDetector.h"                        // nod detector
#include "bayes_filter/bi_ObjectObservation.h"

// SYSTEM INCLUDES
#include <boost/thread/shared_mutex.hpp>            // boost shared mutex
#include <boost/circular_buffer.hpp>                // boost circular buffer

#define LONGTERM_TRACKING

#ifdef __ROSINTEGRATION_FOUND__
#include "ros/ros.h"
#include "perception_msgs/Hold.h"
#endif


#ifdef LONGTERM_TRACKING
// forward declaration
class LongTermTrackerManager;
#endif


/// @brief Main model for the face tracking application
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class MainModel
{
public:
  /// Constructor
  /// @param data_provider Source image provider
  /// @param face_detector_group Group of face detectors to be used by
  /// trackers
  /// @param config Configuration options
  MainModel(ImageProcessing::ip_ImageProvider *data_provider,
            OpenCvPlus::cvp_FaceDetectorGroup *face_detector_group,
            const FaceTrackerConfig& config);
  virtual ~MainModel();
  virtual std::string name() const { return "[MainModel]"; }

  /// Return true if the update went well and false when no new images
  /// are avalable.
  bool update();

  /// Operation used as a slot to communicate if any faces were observed.
  /// Clears the contents of a mutex-protected container that stores
  /// face descriptors and puts there the newly obtained data.
  /// @param faces List of descriptors for detected faces
  void faces_observed(const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces);

  /// Gives read access to local faces cache. This cache is filled during
  /// the update process from the mutex-protected face descriptor storage
  /// using the copy_faces method. It contains the set of faces used in
  /// the current update iteration.
  /// The contents of faces cache might differ from that of the mutex-
  /// protected container as soon as the latter might be accessed
  /// for writing in an asynchronous way.
  /// @return Observed faces cache
  const std::list<OpenCvPlus::cvp_FaceDescriptor> & faces_cache() const;

  const std::list<OpenCvPlus::cvp_BodyDescriptor> & bodies_cache() const;

  /// Sets flag to detect faces internally to true
  void set_detect_faces_flag();

  /// Resets flag to detect faces internally to false
  /// @param timestamp Timestamp of set flag call, to which this reset should
  /// correspond. If this timestamp refers to earlier moment
  /// (an additional call to set flag was performed), nothing happens
  void reset_detect_faces_flag(
                               const boost::posix_time::ptime & timestamp);

  /// Returns the value of the flag to detect faces internally
  /// @param timestamp Timestamp of the call that set the flag
  /// @return True if faces should be detected internally
  bool get_detect_faces_flag(boost::posix_time::ptime & timestamp);

  /// Read accessor to configuration options storage
  /// @return Configuration options storage
  const FaceTrackerConfig& config() const {
    return m_Config;
  }

  ImageProcessing::ip_ImageProvider * data_provider() const {
    return mDataProvider;
  }

  ImageProcessing::ip_ImageProvider * disparity_processor() const {
    return m_DisparityProcessor;
  }

  const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain() const {
    return m_HeadPoseDomain;
  }

  const std::list<BICV::bicv_HeadPoseTracker*>& head_pose_trackers() const {
    return m_HeadPoseTrackers;
  }

#ifndef __VFOA_MODULE_FOUND__
  const VFOA::vfoa_CognitiveVfoaModel * vfoa_model() const {
    return m_VfoaModel;
  }
#else
  VfoaManager * vfoa_model() const {
    return m_VfoaModel;
  }
#endif

  FaceColorModel::FaceColorModel * general_face_color_model() const {
    return m_GeneralFaceColorModel;
  }

  ImageProcessing::ip_Dense2DMotionProcessor *
  dense_motion_processor() const {
    return m_Dense2DMotionProcessor;
  }

  float performance_fps() const;
  float performance_fps2() const;
  float performance_fps3() const;

  const BayesImage::bi_ObjectObservation *
  tracker_manager_observation(BICV::bicv_HeadPoseTracker * tracker) const;

  const NoddingDetector * nod_detector() const {
    return m_NodDetector;
  }

private:

  /// Copies current face set in a thread-safe manner to the provided buffer.
  /// Performs mutex-protected read of face descriptor buffer filled by
  /// faces_observed slot and copies the data to the provided buffer.
  /// @param faces List to copy face descriptors to
  /// @param timestamp Timestamp of the detected faces
  /// @return True if faces were reported (to distinguish between
  ///         no faces observed and no report available)
  bool copy_faces(std::list<OpenCvPlus::cvp_FaceDescriptor> & faces,
                  boost::posix_time::ptime & timestamp);

  /// Resets face detection flag and clears the face detection list, if
  /// no new faces were detected meanwhile.
  /// @param timestamp Timestamp of the detected faces
  /// @return True if no new faces were reported meanwhile
  bool drop_faces(const boost::posix_time::ptime & timestamp);

  void correct_face_coordinates(
                                std::list<OpenCvPlus::cvp_FaceDescriptor> & faces,
                                int width, int height);

  void initialize();
  void initialize_providers(const FaceTrackerConfig & config);
  void initialize_filter(const FaceTrackerConfig & config);

  void invalidate_providers();
  void deinitialize_providers();

  void deinitialize_filter();

  void filter_detected_faces(
                             const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces,
                             std::list<OpenCvPlus::cvp_FaceDescriptor>& init_candidates);

  /// Attempts to initialize a new head pose tracker using the current
  /// face detection and the corresponding time stamp
  /// @param init_candidate Face descriptor to use for initialization
  /// @param timestamp Timestamp of the initialization
  BICV::bicv_HeadPoseTracker * try_initialize_head_pose_tracker(
                                                                const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
                                                                const boost::posix_time::ptime & timestamp);

  void update_colour_model(const std::list<ImageProcessing::ip_RoiWindow>& faces);

  /// initializes face detector statistics data using file names specified
  /// in the config file
  void initialize_face_detector_statistics(const FaceTrackerConfig & config);
  /// deletes face detector statistics data from the model
  void deinitialize_face_detector_statistics();



  OpenCvPlus::cvp_HeadPoseDiscreteDomain m_HeadPoseDomain;
#ifdef __VFOA_MODULE_FOUND__
  VfoaManager * m_VfoaModel;
#else
  VFOA::vfoa_CognitiveVfoaModel * m_VfoaModel;
#endif

  // various data providers
  ImageProcessing::ip_ImageProvider * mDataProvider;
  ImageProcessing::ip_ImageProvider * m_GrayscaleProvider;
  ImageProcessing::ip_GradientImageProcessor * m_GradientProvider;
  ImageProcessing::ip_ColourSkinFeatureProducer * mSkinColourProducer;
  ImageProcessing::ip_ImageProvider * m_DisparityProcessor;

  // face detector group
  OpenCvPlus::cvp_FaceDetectorGroup * m_FaceDetectorGroup;

  // dynamics
  BICV::bicv_HeadPoseLikelihoodModel * m_LikelihoodModel;
  std::list<BICV::bicv_HeadPoseTracker*> m_HeadPoseTrackers;
  BICV::bicv_HeadPoseTracker * m_Initializer;

  // tracker manager to adjust tracker set - add/remove trackers
#ifdef LONGTERM_TRACKING
  LongTermTrackerManager * m_TrackerManager;
#else
  TrackerManager * m_TrackerManager;
#endif

  BICV::bicv_HeadPoseHogModel * m_TrainedHogModel;
  BICV::bicv_HeadPoseSkinModel * m_TrainedSkinModel;
  // used by trackers to get face detection statistics on head pose and bbox
  OpenCvPlus::cvp_FaceDetectorStatisticsStorage * m_FaceDetectorStatisticsStorage;

  FaceColorModel::FaceColorModel * m_GeneralFaceColorModel;

  // flag indicating that external face detector detected new faces
  volatile bool m_NewFacesDetected;
  // time associated to faces from an external face detector
  boost::posix_time::ptime m_NewFacesDetectedTime;
  // cache where external face detector stores detected faces
  std::list<OpenCvPlus::cvp_FaceDescriptor> m_FacesDetected;
  // faces considered during current iteration by the main model
  std::list<OpenCvPlus::cvp_FaceDescriptor> m_FacesCache;
  // bodies considered during current iteration by the main model
  std::list<OpenCvPlus::cvp_BodyDescriptor> m_BodiesCache;
  // synchronization mutex for read/write access to m_FacesDetected
  boost::shared_mutex m_FacesDetectedMutex;

  // flag indicating that face detector should be called internally
  volatile bool m_DetectFacesInternallyFlag;
  // time associated to faces from an external face detector
  boost::posix_time::ptime m_DetectFacesInternallyFlagTime;
  // synchronization mutex for read/write access to
  // m_DetectFacesInternallyFlag
  boost::shared_mutex m_DetectFacesInternallyFlagMutex;

#ifdef __PERFORMANCE_EVALUATION__
  int m_FaceDetectionCounter;
#endif

  // boost::circular_buffer<float> m_FpsValues;
  boost::circular_buffer<boost::posix_time::ptime> m_TimeValues;

  const FaceTrackerConfig& m_Config;

  // dense 2D motion processor
  ImageProcessing::ip_Dense2DMotionProcessor *m_Dense2DMotionProcessor;

  // nod detector
  NoddingDetector *m_NodDetector;

  bool m_IsInitialized;

  // ROS Only: Whether to stop the tracking. This variable is
  // indicating whether the robot is moving (so the input is not
  // reliable and we stop the tracking)
  bool m_hold;

#ifdef __ROSINTEGRATION_FOUND__
  ros::NodeHandle m_nh;
  ros::Subscriber m_hold_sub;
  void hold_received(const perception_msgs::Hold::ConstPtr& msg) { m_hold = msg->hold; }

#endif

};

#endif // __HOGVISUALISER_H__
