// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainModel - main model for the face tracking application
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop
#include <numeric>                                  // accumulate
#include <omp.h>                                    // OpenMP

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistrMixture.h>     // distr mixture
#include <image_processing/ip_DataImageProvider.h>       // file&camera capture
#include <image_processing/ip_GrayscaleImageProvider.h>  // grayscale provider
#include <image_processing/ip_GradientImageProcessor.h>  // gradient processor
#include <image_processing/ip_IntegralGradientHistogramProcessor.h>  // histogram processor
#include <image_processing/ip_SkinColourProcessor.h>   // skin colour processor
#include <image_processing/ip_IntegralSkinMaskProcessor.h> // integral skin provider
#include <image_processing/ip_Dense2DMotionProcessorInria.h> // dense motion processor
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // state2roi converter

#ifdef __STEREO_MATCHER_FOUND__
#include <image_processing/ip_DisparityImageProcessor.h>   // disparity/depth
#endif

// LOCAL INCLUDES
#include "MainModel.h"
#include "LongTermTrackerManager.h"                 // tracker manager

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BICV;
using namespace VFOA;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

// number of cached values for robust FPS calculation
static const int FPS_VALUES_MAXCOUNT = 10;

// parameters for HOG feature model
static const int HISTOGRAM_NBINS = 8;
static const int NUM_BLOCKS_ROW = 2;        // number of blocks per pattern row
static const int NUM_BLOCKS_COL = 2;        // number of blocks per pattern col
static const int NUM_CELLS_IN_BLOCK_ROW = 4; // number of cells per block row
static const int NUM_CELLS_IN_BLOCK_COL = 4; // number of cells per block col

// parameters for skin feature model
static const float SKIN_MASK_VALUE = 255.0f; // value of skin pixels in a mask

#ifdef __PERFORMANCE_EVALUATION__
// number of video frames to skip
static const int FACE_DETECTION_NUM_FRAMES_SKIP = 5;
#endif


MainModel::
MainModel(ImageProcessing::ip_ImageProvider *data_provider,
          cvp_FaceDetectorGroup *face_detector_group,
          const FaceTrackerConfig& config):
  mDataProvider(data_provider),
  m_GrayscaleProvider(0),
  m_GradientProvider(0),
  mSkinColourProducer(0),
  m_DisparityProcessor(0),
  m_FaceDetectorGroup(face_detector_group),
  m_LikelihoodModel(0),
  m_Initializer(0),
  m_TrackerManager(0),
  m_TrainedHogModel(0),
  m_TrainedSkinModel(0),
  m_FaceDetectorStatisticsStorage(0),
  m_GeneralFaceColorModel(0),
  m_NewFacesDetected(false),
  m_DetectFacesInternallyFlag(false),
#ifdef __PERFORMANCE_EVALUATION__
  m_FaceDetectionCounter(0),
#endif
  m_TimeValues(FPS_VALUES_MAXCOUNT),
  m_Config(config),
  m_Dense2DMotionProcessor(0),
  m_IsInitialized(false),
  m_hold(false)
#ifdef __ROSINTEGRATION_FOUND__
  ,m_nh("~") // Don't know what namespace to put
#endif

{
#ifdef __ROSINTEGRATION_FOUND__
  m_hold_sub = m_nh.subscribe("/hold_perception", 10, &MainModel::hold_received, this);
#endif


}


MainModel::
~MainModel()
{
  if(m_NodDetector)
    {
      delete m_NodDetector;
      m_NodDetector = 0;
    }

  if(m_VfoaModel)
    {
      delete m_VfoaModel;
      m_VfoaModel = 0;
    }

  if(m_TrackerManager)
    {
      delete m_TrackerManager;
      m_TrackerManager = 0;
    }

  deinitialize_filter();
  deinitialize_providers();
  deinitialize_face_detector_statistics();

  if (m_GeneralFaceColorModel)
    {
      delete m_GeneralFaceColorModel;
      m_GeneralFaceColorModel = 0;
    }
}


void
MainModel::
initialize_face_detector_statistics(const FaceTrackerConfig& config)
{
  cvp_FaceDetectorStatisticsStorage::ConfigMap fdConfigMap;
  fdConfigMap[CVP_FACEDETECTOR_FACE] = m_Config.m_FrontalFaceStatsFilePath;
  fdConfigMap[CVP_FACEDETECTOR_PROFILE_LEFT] = m_Config.m_ProfileLeftFaceStatsFilePath;
  fdConfigMap[CVP_FACEDETECTOR_PROFILE_RIGHT] = m_Config.m_ProfileRightFaceStatsFilePath;
  m_FaceDetectorStatisticsStorage = new cvp_FaceDetectorStatisticsStorage(fdConfigMap);
}


void
MainModel::
deinitialize_face_detector_statistics()
{
  if(m_FaceDetectorStatisticsStorage)
    {
      delete m_FaceDetectorStatisticsStorage;
      m_FaceDetectorStatisticsStorage = 0;
    }
}


void
MainModel::
initialize_providers(const FaceTrackerConfig& config)
{
  m_GrayscaleProvider = new ip_GrayscaleImageProvider(mDataProvider);
  m_GradientProvider = new ip_GradientImageProcessor(m_GrayscaleProvider);

  ip_HogFeatureParameters hog_fd_params;
  hog_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
  hog_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
  hog_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
  hog_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
  hog_fd_params.mNumHistBins = HISTOGRAM_NBINS;

  ip_SkinFeatureParameters skin_fd_params;
  skin_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
  skin_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
  skin_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
  skin_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
  skin_fd_params.mSkinMaskValue = SKIN_MASK_VALUE;

  ip_ColourSkinFeatureParameters csf_params;
  csf_params.mNumBlocksCol = NUM_BLOCKS_COL;
  csf_params.mNumBlocksRow = NUM_BLOCKS_ROW;
  csf_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
  csf_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
  mSkinColourProducer = new ip_ColourSkinFeatureProducer(mDataProvider,
                                                         csf_params);

  m_Dense2DMotionProcessor =
    new ip_Dense2DMotionProcessorInria(m_GrayscaleProvider,
                                       ip_Dense2DMotionProcessorInriaConfig::getDefault());

#ifdef __STEREO_MATCHER_FOUND__
  m_DisparityProcessor = new ip_DisparityImageProcessor(mDataProvider,
                                                        config.m_StereoCalibrationFile);
#else
  m_DisparityProcessor = 0;
#endif

}

void
MainModel::
invalidate_providers()
{
  if(mDataProvider)
    mDataProvider->invalidate();

  if(m_GrayscaleProvider)
    m_GrayscaleProvider->invalidate();

  if(m_GradientProvider)
    m_GradientProvider->invalidate();

  if(m_Dense2DMotionProcessor)
    m_Dense2DMotionProcessor->invalidate();

  if (m_DisparityProcessor)
    m_DisparityProcessor->invalidate();
}


void
MainModel::
deinitialize_providers()
{
  if(m_Dense2DMotionProcessor)
    {
      delete m_Dense2DMotionProcessor;
      m_Dense2DMotionProcessor = 0;
    }

  if(mSkinColourProducer)
    {
      delete mSkinColourProducer;
      mSkinColourProducer = 0;
    }

  if(m_GradientProvider)
    {
      delete m_GradientProvider;
      m_GradientProvider = 0;
    }

  if(m_GrayscaleProvider)
    {
      delete m_GrayscaleProvider;
      m_GrayscaleProvider = 0;
    }

  if(m_DisparityProcessor)
    {
      delete m_DisparityProcessor;
      m_DisparityProcessor = 0;
    }
}


void
MainModel::
initialize_filter(const FaceTrackerConfig& config)
{
  m_TrainedHogModel =
    bicv_HeadPoseHogModel::load(config.m_TrainedHogModelPath);

  m_TrainedSkinModel =
    bicv_HeadPoseSkinModel::load(config.m_TrainedSkinModelPath);

  m_LikelihoodModel = new bicv_HeadPoseLikelihoodModel(*m_TrainedHogModel,
                                                       *m_TrainedSkinModel,
                                                       m_HeadPoseDomain);

  ip_HogFeatureParameters hog_fd_params;
  hog_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
  hog_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
  hog_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
  hog_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
  hog_fd_params.mNumHistBins = HISTOGRAM_NBINS;

  ip_SkinFeatureParameters skin_fd_params;
  skin_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
  skin_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
  skin_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
  skin_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
  skin_fd_params.mSkinMaskValue = SKIN_MASK_VALUE;

  // provide a separate instance of dynamic model to each tracker!
  bicv_HeadPoseMixtureDynamicModel * tracker_dynamic_model =
    new bicv_HeadPoseMixtureDynamicModel(m_Config.m_DynamicModelConfig,
                                         m_HeadPoseDomain, *m_FaceDetectorStatisticsStorage);

  ip_Dense2DMotionProcessorInria * motion_proc =
    dynamic_cast<ip_Dense2DMotionProcessorInria*>(m_Dense2DMotionProcessor);

  m_Initializer =
    new bicv_HeadPoseTracker(tracker_dynamic_model,
                             m_LikelihoodModel,
                             mDataProvider,
                             m_GrayscaleProvider,
                             m_GradientProvider, mSkinColourProducer,
                             motion_proc, m_GeneralFaceColorModel,
                             m_DisparityProcessor, m_TrainedSkinModel,
                             hog_fd_params, skin_fd_params,
                             m_HeadPoseDomain,
                             m_FaceDetectorStatisticsStorage,
                             config.m_NumTrackerParticles);
}


void
MainModel::
deinitialize_filter()
{
  BOOST_FOREACH(bicv_HeadPoseTracker * tracker, m_HeadPoseTrackers)
    {
      if(tracker)
        delete tracker;
    }
  m_HeadPoseTrackers.clear();

  if(m_Initializer)
    {
      delete m_Initializer;
      m_Initializer = 0;
    }

  if(m_LikelihoodModel)
    {
      delete m_LikelihoodModel;
      m_LikelihoodModel = 0;
    }

  if(m_TrainedSkinModel)
    {
      delete m_TrainedSkinModel;
      m_TrainedSkinModel = 0;
    }

  if(m_TrainedHogModel)
    {
      delete m_TrainedHogModel;
      m_TrainedHogModel = 0;
    }
}

void
MainModel::
faces_observed(const std::list<cvp_FaceDescriptor> & faces)
{
  m_FacesDetectedMutex.lock();
  m_NewFacesDetected = true;
  m_NewFacesDetectedTime = boost::posix_time::microsec_clock::local_time();

  m_FacesDetected.clear();
  copy(faces.begin(), faces.end(), back_inserter(m_FacesDetected));
  m_FacesDetectedMutex.unlock();
}


bool
MainModel::copy_faces(std::list<cvp_FaceDescriptor> & faces,
                      boost::posix_time::ptime & timestamp)
{
  m_FacesDetectedMutex.lock_shared();
  copy(m_FacesDetected.begin(), m_FacesDetected.end(), back_inserter(faces));
  timestamp = m_NewFacesDetectedTime;
  bool result = m_NewFacesDetected;
  m_FacesDetectedMutex.unlock_shared();
  return result;
}


bool
MainModel::
drop_faces(const boost::posix_time::ptime & timestamp)
{
  m_FacesDetectedMutex.lock_shared();
  bool result = (m_NewFacesDetectedTime == timestamp);
  if (result) {
    m_FacesDetected.clear();
    m_NewFacesDetected = false;
  }
  m_FacesDetectedMutex.unlock_shared();
  return result;
}


void
MainModel::
set_detect_faces_flag()
{
  m_DetectFacesInternallyFlagMutex.lock();
  m_DetectFacesInternallyFlag = true;
  m_DetectFacesInternallyFlagTime =
    boost::posix_time::microsec_clock::local_time();
  m_DetectFacesInternallyFlagMutex.unlock();
}


void
MainModel::
reset_detect_faces_flag(const boost::posix_time::ptime & timestamp)
{
  m_DetectFacesInternallyFlagMutex.lock();
  bool should_reset = (m_DetectFacesInternallyFlagTime == timestamp);
  if (should_reset) {
    m_DetectFacesInternallyFlag = false;
  }
  m_DetectFacesInternallyFlagMutex.unlock();
}


bool
MainModel::
get_detect_faces_flag(boost::posix_time::ptime & timestamp)
{
  m_DetectFacesInternallyFlagMutex.lock_shared();
  bool result = m_DetectFacesInternallyFlag;
  timestamp = m_DetectFacesInternallyFlagTime;
  m_DetectFacesInternallyFlagMutex.unlock_shared();
  return result;
}


void
MainModel::
initialize()
{
  if(m_IsInitialized) return;

  std::cout << name() << " Initializing" << std::endl;

  m_GeneralFaceColorModel =
    new FaceColorModel::FaceColorModel(m_Config.m_FaceColorModelConfig);

  IplImage *buf = mDataProvider->image_buffer();

  if(!buf) return;

  // std::cout << "From MainModel image of size "
  //           << buf->width << "x" << buf->height
  //           << std::endl;

  m_GeneralFaceColorModel->prepare(mDataProvider->image_buffer());

  initialize_face_detector_statistics(m_Config);
  initialize_providers(m_Config);
  initialize_filter(m_Config);

#ifdef __VFOA_MODULE_FOUND__
  m_VfoaModel = new VfoaManager();
#else
  m_VfoaModel = new vfoa_CognitiveVfoaModel(m_HeadPoseDomain,
                                            mDataProvider->image_buffer()->width,
                                            mDataProvider->image_buffer()->height);
#endif


#ifdef LONGTERM_TRACKING
  m_TrackerManager = new LongTermTrackerManager(mDataProvider, m_Config);
#else
  m_TrackerManager = new TrackerManager(mDataProvider, m_Config);
#endif

  m_NodDetector = (m_Config.m_NodDetectorFilePathProvided ?
                   new NoddingDetector(m_Config.m_NodDetectorFilePath) :
                   0);

  m_IsInitialized = true;
}


bool
MainModel::
update()
{
  boost::posix_time::ptime update_start =
    boost::posix_time::microsec_clock::local_time();

  m_TimeValues.push_back(update_start);

  invalidate_providers();

  // IplImage *input_image = mDataProvider->image();
  IplImage *input_image = NULL;
  try
    {
      input_image = mDataProvider->image();
    }
  catch(...)
    {
      std::cout << name() << " Caught exception in getting image" << std::endl;
    }

  if(!input_image)
    {
      // std::cout << name() << " No image from provider... ";
      if(mDataProvider->should_wait())
        {
          // std::cout << "Waiting." << std::endl;
          return true;
        }
      else
        {
          // std::cout << "Stopping." << std::endl;
          return false;
        }
    }

  // Initialize after first image is read so that image_buffer is
  // valid in the data provider
  if(!m_IsInitialized) initialize();

  ip_RoiWindow all_image_roi =
    ip_RoiWindow::from_CvRect(cvRect(0, 0,
                                     input_image->width,
                                     input_image->height));

  // get and cache the most recent processor images
  /* IplImage * grayscale_image = */ m_GrayscaleProvider->image();
  /* IplImage * gradient_image = */ m_GradientProvider->image(all_image_roi);

  // get newly detected faces reported by an external face detector
  m_FacesCache.clear();
  m_BodiesCache.clear();
  boost::posix_time::ptime face_detection_timestamp;
  bool face_detector_reported =
    copy_faces(m_FacesCache, face_detection_timestamp);
  drop_faces(face_detection_timestamp);

#ifndef __PERFORMANCE_EVALUATION__
// #pragma omp parallel sections
    {
#endif

#ifdef __STEREO_MATCHER_FOUND__
#ifndef __PERFORMANCE_EVALUATION__
// #pragma omp section
    {
#endif
    ip_DisparityImageProcessor * disparity_processor =
            dynamic_cast<ip_DisparityImageProcessor*>(m_DisparityProcessor);
    if (disparity_processor) {
//        cout << "Disparity update, thread " << omp_get_thread_num() << endl << flush;
        disparity_processor->image();
    }
#ifndef __PERFORMANCE_EVALUATION__
    }
#endif
#endif

#ifdef __PERFORMANCE_EVALUATION__
    if (!(++m_FaceDetectionCounter % FACE_DETECTION_NUM_FRAMES_SKIP)) {
        m_FaceDetectionCounter = 0;
        const list<cvp_FaceDescriptor>& faces =
            m_FaceDetectorGroup->detect_faces_next(input_image,
                                                   mDataProvider->time());
        copy(faces.begin(), faces.end(), back_inserter(m_FacesCache));
        reset_detect_faces_flag(face_detection_timestamp);
        face_detector_reported = true;
    }
#else
   // #pragma omp master
    {
   // cout << "Face detector update, thread " << omp_get_thread_num() << endl << flush;
    // check if should use internal face detector
    if (get_detect_faces_flag(face_detection_timestamp))
      {
        const list<cvp_FaceDescriptor>& faces =
          m_FaceDetectorGroup->detect_faces_next(input_image,
                                                 mDataProvider->time());

        copy(faces.begin(), faces.end(), back_inserter(m_FacesCache));
        reset_detect_faces_flag(face_detection_timestamp);
        face_detector_reported = true;

        const list<cvp_BodyDescriptor>& bodies =
          m_FaceDetectorGroup->cached_bodies();
        copy(bodies.begin(), bodies.end(), back_inserter(m_BodiesCache));
      }
    }
#endif

#ifndef __PERFORMANCE_EVALUATION__
    // #pragma omp section
    {
//   cout << "Motion estimation update, thread " << omp_get_thread_num() << endl << flush;
#endif
    // recomputes pyramides if necessary
    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    dense_motion_processor());
    motion_proc->image();
#ifndef __PERFORMANCE_EVALUATION__
    }
    }
#endif

    // correct face coordinates if face detector and tracker input image sizes
    // are different; assign face detections to trackers
    correct_face_coordinates(m_FacesCache, input_image->width, input_image->height);
    std::list<cvp_FaceDescriptor> init_candidates;
    if (!m_FacesCache.empty()) {
        filter_detected_faces(m_FacesCache, init_candidates);
    }

    // update all current trackers
//    cout << "Start update trackers..." << endl;
    const vector<bicv_HeadPoseTracker*> trackers(m_HeadPoseTrackers.begin(),
            m_HeadPoseTrackers.end());
    const unsigned trackers_num = trackers.size();
    unsigned tracker_idx;
#ifndef __PERFORMANCE_EVALUATION__
    unsigned thread_idx;
#pragma omp parallel private(tracker_idx, thread_idx) //num_threads(3)
    {
       thread_idx = omp_get_thread_num();
    #pragma omp for
#endif
    for (tracker_idx = 0; tracker_idx < trackers_num; ++tracker_idx) {
        trackers[tracker_idx]->iterate();
        // cout << "Thread " << thread_idx << " updated tracker "
        //      << trackers[tracker_idx] << endl;
    }
#ifndef __PERFORMANCE_EVALUATION__
    }
#endif
//    cout << "End update trackers..." << endl;

    // manage the set of trackers - select trackers to add/remove
//    cout << "Start manage trackers..." << endl;
    list<bicv_HeadPoseTracker *> trackers_to_remove;
    list<cvp_FaceDescriptor> trackers_to_add;
    m_TrackerManager->manage_trackers(face_detector_reported,
            init_candidates, m_HeadPoseTrackers,
            trackers_to_remove, trackers_to_add);
//    cout << "End manage trackers..." << endl;


    std::set<const bicv_HeadPoseTracker*> already_removed_trackers;

    if(m_hold && m_HeadPoseTrackers.size() > 0)
      {
        std::cout << "[MainModel] Holding perception. Remove "
                  << m_HeadPoseTrackers.size() << " trackers: ";
        BOOST_FOREACH(bicv_HeadPoseTracker *tracker, m_HeadPoseTrackers) {
          trackers_to_remove.push_back(tracker);
          std::cout << tracker->id() << " ";
        }
        std::cout << std::endl;
      }

    // remove obsolete trackers
    BOOST_FOREACH(bicv_HeadPoseTracker *tracker, trackers_to_remove) {
#ifndef __VFOA_MODULE_FOUND__
        m_VfoaModel->remove_detected_object(tracker);
#endif
        if(already_removed_trackers.count(tracker)==0)
          {
            m_HeadPoseTrackers.remove(tracker);
            already_removed_trackers.insert(tracker);
            delete tracker;
          }
        else
          {
            std::cout << "[MainModel] Already removed tracker "
                      << tracker << std::endl;
          }
    }

    // If we hold the perception because input is not reliable

    // std::cout << "#trackers " << m_HeadPoseTrackers.size() << std::endl;

    // if(m_hold)
    //   {
    //     std::cout << "[MainModel] Holding perception and remove all " << m_HeadPoseTrackers.size() << std::endl;
    //     BOOST_FOREACH(bicv_HeadPoseTracker *tracker, m_HeadPoseTrackers) {

    //       std::cout << "#trackers " << m_HeadPoseTrackers.size()
    //                 << " before removing " << tracker->id() << std::endl;

    //       std::cout << "  id " << tracker->id() << std::endl;
    //       m_HeadPoseTrackers.remove(tracker);
    //       delete tracker;

    //       std::cout << "#trackers " << m_HeadPoseTrackers.size()
    //                 << " after removing " << tracker->id() << std::endl;

    //     }
    //   }

    // try adding new trackers
    if(!trackers_to_add.empty() && !m_hold)
    // if(!trackers_to_add.empty())
      {
        bicv_HeadPoseTracker *new_tracker = try_initialize_head_pose_tracker
          (trackers_to_add.front(), mDataProvider->time());
        if(new_tracker)
          {
            m_HeadPoseTrackers.push_back(new_tracker);
#ifndef __VFOA_MODULE_FOUND__
            m_VfoaModel->add_detected_object(new_tracker);
#endif
          }
      } // if

#ifdef __VFOA_MODULE_FOUND__
    object_list_t object_list;
    person_list_t person_list;
    static float const DEGREES_TO_RADIANS_CONST =  3.1415926536 / 180;
    ostringstream person_name_oss;

//    cout << "Tracker list: (";
    BOOST_FOREACH(bicv_HeadPoseTracker * tracker, m_HeadPoseTrackers) {
        person_info_t pPerson;
        pPerson.id = tracker->id();
        person_name_oss.str("");
        person_name_oss << "Person " << pPerson.id;
        CvScalar coords_3d = tracker->scene_coordinates_3D();
        pPerson.loc.x = -coords_3d.val[0];
        pPerson.loc.y = coords_3d.val[2];
        pPerson.loc.z = -coords_3d.val[1];
        cvp_HeadPoseDiscreteDomain::HeadPose head_pose =
            mean(tracker->particle_distribution())
            .m_HeadPoseParamsCur.m_HeadPose;
        pPerson.head_pose.pan = -head_pose.pan() * DEGREES_TO_RADIANS_CONST;
        pPerson.head_pose.tilt = head_pose.tilt() * DEGREES_TO_RADIANS_CONST;
        pPerson.head_pose.roll = head_pose.roll() * DEGREES_TO_RADIANS_CONST;
        pPerson.name = person_name_oss.str();
        person_list.push_back(pPerson);
//        cout << pPerson.id << ", ";
    }
//    cout << ")" << endl;
//    cout << person_list << endl;

    object_info_t pNao;
    pNao.id = 1;
    pNao.name = "Robot";
    pNao.loc.x = 0;
    pNao.loc.y = 0;
    pNao.loc.z = 0;
    object_list.push_back(pNao);

    // object_info_t pTablet;
    // pTablet.id = 2;
    // pTablet.name = "Tablet";
    // pTablet.loc.x = 1000*-0.0170669199141;
    // pTablet.loc.y = 1000*-0.00224424506657;
    // pTablet.loc.z = 1000*-0.309832935109;
    // object_list.push_back(pTablet);


//    cout << "Object list: (" << pNao.id << ")" << endl;
//    cout << object_list << endl;

    m_VfoaModel->UpdateTargetList(object_list, person_list, mDataProvider->time());
#endif

    boost::posix_time::ptime update_end =
      boost::posix_time::microsec_clock::local_time();

    // std::cout << "[MainModel] Provider of " << to_simple_string(mDataProvider->time()) << std::endl;
    // std::cout << "[MainModel] End update at " << to_simple_string(update_end) << std::endl;

    boost::posix_time::time_duration update_time = update_end - update_start;

    long update_millis = update_time.total_milliseconds();
    if (update_millis <= 0) {
        update_millis = 1;
    }
    // m_FpsValues.push_back(1000.0 / update_millis);

    return true;
}

void MainModel::correct_face_coordinates(
        std::list<cvp_FaceDescriptor> & faces,
        int width, int height) {
    BOOST_FOREACH(cvp_FaceDescriptor& fd, faces) {
        float coeff_x = static_cast<float>(fd.m_ImageWidth) /
                width;
        float coeff_y = static_cast<float>(fd.m_ImageHeight) /
                height;

        fd.m_FaceRegion.width /= coeff_x;
        fd.m_FaceRegion.height /= coeff_y;
        fd.m_FaceRegion.x /= coeff_x;
        fd.m_FaceRegion.y /= coeff_y;
        fd.m_ImageWidth = width;
        fd.m_ImageHeight = height;

    }
}

// float MainModel::performance_fps() const {
//     if (m_FpsValues.empty()) {
//         return 0;
//     } else {
//         return accumulate(m_FpsValues.begin(), m_FpsValues.end(), 0.0) /
//                 m_FpsValues.size();
//     }
// }

float MainModel::performance_fps() const {
  float fps = 0;
  int nValues = static_cast<int>(m_TimeValues.size());

  typedef boost::circular_buffer<boost::posix_time::ptime>::const_iterator Iterator;

  if(nValues>1)
    {
      Iterator beg = m_TimeValues.begin();
      Iterator end = m_TimeValues.end();
      end--;
      boost::posix_time::time_duration update_time = *(end) - *(beg);
      long msecs = update_time.total_milliseconds();
      fps = static_cast<float>(nValues)/static_cast<float>(msecs)*1000.0;
    }
  return fps;
}


const BayesImage::bi_ObjectObservation *
MainModel::tracker_manager_observation(
        BICV::bicv_HeadPoseTracker * tracker) const {
    return m_TrackerManager->get_observation(tracker);
} // tracker_manager_observation


void MainModel::update_colour_model(const std::list<ip_RoiWindow>& faces) {
//    const vector<ip_SkinTemplate>& templates =
//            m_TrainedSkinModel->mean_templates();
//    ip_SkinTemplate avg_templ = *templates.begin();
//    avg_templ = accumulate(templates.begin() + 1, templates.end(), avg_templ);
//    avg_templ /= templates.size();
//
//    ip_SkinColourModel model;
//
//    BOOST_FOREACH(ip_RoiWindow roi, faces) {
//        ip_ColourSkinTemplate tt =
//                m_ColourSkinFeatureProducer->compute_feature(roi);
//        tt.mBlueColourTemplate *= avg_templ;
//        tt.mRedColourTemplate *= avg_templ;
//        tt.mGreenColourTemplate *= avg_templ;
//        tt.mBlueColourTemplate += tt.mRedColourTemplate + tt.mGreenColourTemplate;
//        tt.mGreenColourTemplate /= tt.mBlueColourTemplate;
//        tt.mRedColourTemplate /= tt.mBlueColourTemplate;
//        model.mGreenMean += tt.mGreenColourTemplate.sum();
//        model.mGreenVar +=
//                (tt.mGreenColourTemplate * tt.mGreenColourTemplate).sum();
//        model.mRedMean += tt.mGreenColourTemplate.sum();
//        model.mRedVar += (tt.mRedColourTemplate * tt.mRedColourTemplate).sum();
//        model.mGreenRedCovar +=
//                (tt.mGreenColourTemplate * tt.mRedColourTemplate).sum();
//    }
//    model.mGreenMean /= (faces.size() * avg_templ.sum());
//    model.mRedMean   /= (faces.size() * avg_templ.sum());
//    model.mGreenVar  = model.mGreenVar / (faces.size() * avg_templ.sum()) -
//            model.mGreenMean * model.mGreenMean;
//    model.mRedVar = model.mRedVar / (faces.size() * avg_templ.sum()) -
//            model.mRedMean * model.mRedMean;
//    model.mGreenRedCovar = model.mGreenRedCovar /
//            (faces.size() * avg_templ.sum()) -
//            model.mGreenMean * model.mRedMean;
//    mSkinMaskProvider->update_colour_model(model);
}

const std::list<cvp_FaceDescriptor>&
MainModel::
faces_cache() const
{
  return m_FacesCache;
}

const std::list<OpenCvPlus::cvp_BodyDescriptor>&
MainModel::
bodies_cache() const
{
  return m_BodiesCache;
}

void
MainModel::
filter_detected_faces(const std::list<cvp_FaceDescriptor>& faces,
                      std::list<cvp_FaceDescriptor>& init_candidates)
{
  bicv_HeadPoseARTrackerState mean_state;
  ip_RoiWindow mean_tracker_roi;
  bool consider_roi_flag;

  int face_id = 0;
  BOOST_FOREACH(cvp_FaceDescriptor fd, faces)
    {
      consider_roi_flag = true;
      BOOST_FOREACH(bicv_HeadPoseTracker *tracker, m_HeadPoseTrackers)
        {
          mean_state = mean(tracker->particle_distribution());
          mean_tracker_roi = HpParams2RoiConverter::
            hpparams2roi(mean_state.m_HeadPoseParamsCur);

          float intersection = intersection_area
            (ip_RoiWindow::from_CvRect(fd.m_FaceRegion), mean_tracker_roi);

          // std::cout << "intersection " << intersection << std::endl;

          // Original code, dropping an image if it overlaps with a
          // tracker causing nearby people not to be tracked together
          if(intersection > 0)
            {
              tracker->observe_face(fd);
              tracker->set_face_idx(face_id);
              consider_roi_flag = false;
            }

          // float intersection2 = intersection_area
          //   (ip_RoiWindow::from_CvRect(fd.m_FaceRegion), mean_tracker_roi)/
          //   static_cast<float>(area(mean_tracker_roi));

          // float intersection3 = intersection_area
          //   (ip_RoiWindow::from_CvRect(fd.m_FaceRegion), mean_tracker_roi)/
          //   static_cast<float>(area(ip_RoiWindow::from_CvRect(fd.m_FaceRegion)));

          // // Use 0.3 threshold like in face detection
          // if(intersection3 > 0.3)
          //   {
          //     tracker->observe_face(fd);
          //     tracker->set_face_idx(face_id);
          //     consider_roi_flag = false;
          //   }

          // std::cout << "consider " << consider_roi_flag
          //           << " " << intersection
          //           << " " << intersection2
          //           << " " << intersection3 << std::endl;
        }

      if(consider_roi_flag)
        {
          init_candidates.push_back(fd);
        }

      face_id++;

    }
}

BICV::bicv_HeadPoseTracker * MainModel::try_initialize_head_pose_tracker(
        const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
        const boost::posix_time::ptime & timestamp) {

    // bicv_HeadPoseARTrackerState mode_state = mode(m_Initializer->particle_distribution());

    ip_HogFeatureParameters hog_fd_params;
    hog_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
    hog_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
    hog_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
    hog_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
    hog_fd_params.mNumHistBins = HISTOGRAM_NBINS;

    ip_SkinFeatureParameters skin_fd_params;
    skin_fd_params.mNumBlocksCol = NUM_BLOCKS_COL;
    skin_fd_params.mNumBlocksRow = NUM_BLOCKS_ROW;
    skin_fd_params.mNumCellsInBlockCol = NUM_CELLS_IN_BLOCK_COL;
    skin_fd_params.mNumCellsInBlockRow = NUM_CELLS_IN_BLOCK_ROW;
    skin_fd_params.mSkinMaskValue =SKIN_MASK_VALUE;

    // provide a separate instance of face colour model to each tracker!
    // it should be adapted while tracking
    FaceColorModel::FaceColorModel * tracker_colour_model =
            new FaceColorModel::FaceColorModel(
                    m_Config.m_FaceColorModelConfig);

    IplImage * source_img = mDataProvider->image();
    tracker_colour_model->prepare(source_img);

    // provide a separate instance of dynamic model to each tracker!
    bicv_HeadPoseMixtureDynamicModel * tracker_dynamic_model =
            new bicv_HeadPoseMixtureDynamicModel(
                    m_Config.m_DynamicModelConfig,
                    m_HeadPoseDomain, *m_FaceDetectorStatisticsStorage);

    // initialize tracker with motion processor
    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    m_Dense2DMotionProcessor);

    bicv_HeadPoseTracker * tracker = new bicv_HeadPoseTracker(
            tracker_dynamic_model,
            m_LikelihoodModel, mDataProvider, m_GrayscaleProvider,
            m_GradientProvider, mSkinColourProducer, motion_proc,
            tracker_colour_model, m_DisparityProcessor, m_TrainedSkinModel,
            hog_fd_params, skin_fd_params, m_HeadPoseDomain,
            m_FaceDetectorStatisticsStorage,
            m_Config.m_NumTrackerParticles);
//        cout << "Initialized tracker " << tracker << endl;

    tracker->init(init_candidate, timestamp);

    tracker->face_color_model()->adapt_to(source_img,
            init_candidate.m_FaceRegion);

    return tracker;

}
