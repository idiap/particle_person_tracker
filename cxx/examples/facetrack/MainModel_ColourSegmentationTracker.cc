// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainModel_ColourSegmentationTracker - main model for the colour segmentation
//                                       tracking application
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop
#include <numeric>                                  // accumulate

// PROJECT INCLUDES
#include <image_processing/ip_DataImageProvider.h>              // file&camera capture
#include <image_processing/ip_GrayscaleImageProvider.h>         // grayscale provider
#include <image_processing/ip_Dense2DMotionProcessorInria.h>
#include <bayes_image/bicv_CsMixtureDynamicModel.h>             //
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // state2roi converter
#include <bayes_filter/bf_ConditionalDistrMixture.h>            // distr mixture

#ifdef __STEREO_MATCHER_FOUND__
#include <image_processing/ip_DisparityImageProcessor.h>   // disparity/depth
#endif

// LOCAL INCLUDES
#include "MainModel_ColourSegmentationTracker.h"       // declaration of this
#include "LongTermTrackerManager_ColourSegmentation.h" // declaration of this

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BICV;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

static const float TRANSLATION_X_STDDEV = 5.0;
static const float TRANSLATION_Y_STDDEV = 5.0;
static const float SCALE_STDDEV = 0.005f;
static const float EXCENTRICITY_STDDEV = 0.001f;

static const int FPS_VALUES_MAXCOUNT = 10;

#ifdef __PERFORMANCE_EVALUATION__
// number of video frames to skip
static const int FACE_DETECTION_NUM_FRAMES_SKIP = 10;
#endif

/////////////////////////////// PUBLIC ///////////////////////////////////////

MainModel_ColourSegmentationTracker::MainModel_ColourSegmentationTracker(
        ImageProcessing::ip_ImageProvider * data_provider,
        cvp_FaceDetectorGroup * face_detector_group,
        const FaceTrackerConfig & config) :
            m_DataProvider(data_provider),
            m_FaceDetectorGroup(face_detector_group),
            m_TrackerManager(0),
            m_GeneralFaceColorModel(0),
            m_NewFacesDetected(false),
            m_DetectFacesInternallyFlag(false),
#ifdef __PERFORMANCE_EVALUATION__
            m_FaceDetectionCounter(0),
#endif
            m_FpsValues(FPS_VALUES_MAXCOUNT),
            m_Config(config),
            m_Dense2DMotionProcessor(0) {

    m_GeneralFaceColorModel = new FaceColorModel::FaceColorModel(
        m_Config.m_FaceColorModelConfig);
    m_GeneralFaceColorModel->prepare(m_DataProvider->image_buffer());

    initialize_providers(config);
    initialize_filter(config);

#ifdef LONGTERM_TRACKING
    m_TrackerManager = new LongTermTrackerManager_ColourSegmentation(
            data_provider, m_Config);
#else
    m_TrackerManager = new ColourSegmentationTrackerManager(data_provider, m_Config);
#endif

} // MainModel_ColourSegmentationTracker

MainModel_ColourSegmentationTracker::~MainModel_ColourSegmentationTracker() {

    delete m_GeneralFaceColorModel; m_GeneralFaceColorModel = 0;

    delete m_TrackerManager; m_TrackerManager = 0;

    deinitialize_filter();
    deinitialize_providers();

} // ~MainModel_ColourSegmentationTracker

void MainModel_ColourSegmentationTracker::set_detect_faces_flag() {
    //BEGIN: THREAD-SAFE
    m_DetectFacesInternallyFlagMutex.lock();
    m_DetectFacesInternallyFlag = true;
    m_DetectFacesInternallyFlagTime =
            boost::posix_time::microsec_clock::local_time();
    m_DetectFacesInternallyFlagMutex.unlock();
    //END: THREAD-SAFE
} // set_detect_faces_flag

void MainModel_ColourSegmentationTracker::reset_detect_faces_flag(
    const boost::posix_time::ptime & timestamp) {
    //BEGIN: THREAD-SAFE
    m_DetectFacesInternallyFlagMutex.lock();
    bool should_reset = (m_DetectFacesInternallyFlagTime == timestamp);
    if (should_reset) {
        m_DetectFacesInternallyFlag = false;
    }
    m_DetectFacesInternallyFlagMutex.unlock();
    //END: THREAD-SAFE
} // reset_detect_faces_flag

bool MainModel_ColourSegmentationTracker::get_detect_faces_flag(
        boost::posix_time::ptime & timestamp) {
    m_DetectFacesInternallyFlagMutex.lock_shared();
    bool result = m_DetectFacesInternallyFlag;
    timestamp = m_DetectFacesInternallyFlagTime;
    m_DetectFacesInternallyFlagMutex.unlock_shared();
    return result;

} // get_detect_faces_flag

void MainModel_ColourSegmentationTracker::update() {

    boost::posix_time::ptime update_start =
            boost::posix_time::microsec_clock::local_time();

//    cout << "Updating main model..." << update_start << endl;

    invalidate_providers();

    // get and cache the most recent input image
    IplImage * input_image = m_DataProvider->image();

    // get and cache the most recent processor images
    /* IplImage * grayscale_image = */ m_GrayscaleProvider->image();

    // get newly detected faces reported by an external face detector
    m_FacesCache.clear();
    boost::posix_time::ptime face_detection_timestamp;
    bool face_detector_reported =
        copy_faces(m_FacesCache, face_detection_timestamp);
    drop_faces(face_detection_timestamp);

#ifdef __PERFORMANCE_EVALUATION__
    if (!(++m_FaceDetectionCounter % FACE_DETECTION_NUM_FRAMES_SKIP)) {
        m_FaceDetectionCounter = 0;
        const list<cvp_FaceDescriptor>& faces =
            m_FaceDetectorGroup->detect_faces_next(
                    input_image, m_DataProvider->time());
        copy(faces.begin(), faces.end(), back_inserter(m_FacesCache));
        reset_detect_faces_flag(face_detection_timestamp);
        face_detector_reported = true;
    }
#else
//    cout << "Face detector update, thread " << omp_get_thread_num() << endl << flush;
    // check if should use internal face detector
    if (get_detect_faces_flag(face_detection_timestamp)) {
        const list<cvp_FaceDescriptor>& faces =
            m_FaceDetectorGroup->detect_faces_next(
                    input_image, m_DataProvider->time());
        copy(faces.begin(), faces.end(), back_inserter(m_FacesCache));
        reset_detect_faces_flag(face_detection_timestamp);
        face_detector_reported = true;
    }
#endif

    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    dense_motion_processor());
    // recomputes pyramides if necessary
    motion_proc->image();

    // correct face coordinates if face detector and tracker input image sizes
    // are different; assign face detections to trackers
    correct_face_coordinates(m_FacesCache, input_image->width, input_image->height);
    std::list<cvp_FaceDescriptor> init_candidates;
    if (!m_FacesCache.empty()) {
        filter_detected_faces(m_FacesCache, init_candidates);
    }

    // update all current trackers
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, m_Trackers) {
        tracker->iterate();
    }

    // manage the set of trackers - select trackers to add/remove
    list<bicv_ColourSegmentationTracker *> trackers_to_remove;
    list<cvp_FaceDescriptor> trackers_to_add;
#ifdef LONGTERM_TRACKING
    m_TrackerManager->manage_trackers(face_detector_reported,
        init_candidates, m_Trackers, trackers_to_remove, trackers_to_add);
#else
    m_TrackerManager->manage_trackers(
        init_candidates, m_Trackers, trackers_to_remove, trackers_to_add);
#endif

    // remove obsolete trackers
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers_to_remove) {
        m_Trackers.remove(tracker);
        delete tracker;
    }

    // try adding new trackers
    if (!trackers_to_add.empty()) {
        bicv_ColourSegmentationTracker * new_tracker =
            try_initialize_head_pose_tracker(trackers_to_add.front(),
                m_DataProvider->time());
        if (new_tracker) {
            m_Trackers.push_back(new_tracker);
        }
    }

    boost::posix_time::ptime update_end =
            boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration update_time =
            update_end - update_start;
    long update_millis = update_time.total_milliseconds();
    if (update_millis <= 0) {
        update_millis = 1;
    }
    m_FpsValues.push_back(1000.0 / update_millis);

} // update

void MainModel_ColourSegmentationTracker::faces_observed(
        const std::list<OpenCvPlus::cvp_FaceDescriptor> & faces) {
    //BEGIN: THREAD-SAFE
//    cout << "Faces observed, adding " << faces.size() << " faces to queue... ";
    m_FacesDetectedMutex.lock();
    m_NewFacesDetected = true;
    m_NewFacesDetectedTime = boost::posix_time::microsec_clock::local_time();

    m_FacesDetected.clear();
    copy(faces.begin(), faces.end(), back_inserter(m_FacesDetected));
    m_FacesDetectedMutex.unlock();
//    cout << "Done!" << endl;
    //END: THREAD-SAFE
} // faces_observed

bool MainModel_ColourSegmentationTracker::copy_faces(
        std::list<OpenCvPlus::cvp_FaceDescriptor> & faces,
        boost::posix_time::ptime & timestamp) {
    //BEGIN: THREAD-SAFE
//    cout << "Faces requested, retrieving " << m_FacesDetected.size() << " faces from queue... ";
    m_FacesDetectedMutex.lock_shared();
//    cout << "copying to buffer... ";
    copy(m_FacesDetected.begin(), m_FacesDetected.end(), back_inserter(faces));
//    cout << "Done!";
//    cout << endl;
    timestamp = m_NewFacesDetectedTime;
    bool result = m_NewFacesDetected;
    m_FacesDetectedMutex.unlock_shared();
    return result;
    //END: THREAD-SAFE
} // copy_faces

bool MainModel_ColourSegmentationTracker::drop_faces(
        const boost::posix_time::ptime & timestamp) {
    //BEGIN: THREAD-SAFE
    m_FacesDetectedMutex.lock_shared();
    bool result = (m_NewFacesDetectedTime == timestamp);
    if (result) {
        m_FacesDetected.clear();
        m_NewFacesDetected = false;
    }
    m_FacesDetectedMutex.unlock_shared();
    return result;
    //END: THREAD-SAFE
} // drop_faces

const std::list<OpenCvPlus::cvp_FaceDescriptor> &
MainModel_ColourSegmentationTracker::faces_cache() const {
    return m_FacesCache;
} // faces_cache

void MainModel_ColourSegmentationTracker::correct_face_coordinates(
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

float MainModel_ColourSegmentationTracker::performance_fps() const {
    if (m_FpsValues.empty()) {
        return 0;
    } else {
        return accumulate(m_FpsValues.begin(), m_FpsValues.end(), 0.0) /
                m_FpsValues.size();
    }
} // performance_fps

const BayesImage::bi_ObjectObservation *
MainModel_ColourSegmentationTracker::tracker_manager_observation(
        BICV::bicv_ColourSegmentationTracker * tracker) const {
    return m_TrackerManager->get_observation(tracker);
} // tracker_manager_observation

/////////////////////////////// PRIVATE //////////////////////////////////////

void MainModel_ColourSegmentationTracker::initialize_providers(
    const FaceTrackerConfig & config) {

    m_GrayscaleProvider = new ip_GrayscaleImageProvider(m_DataProvider);
    m_Dense2DMotionProcessor = new ip_Dense2DMotionProcessorInria(
        m_GrayscaleProvider,
        ip_Dense2DMotionProcessorInriaConfig::getDefault());

#ifdef __STEREO_MATCHER_FOUND__
    m_DisparityProcessor = new ip_DisparityImageProcessor(m_DataProvider,
            config.m_StereoCalibrationFile);
#else
    m_DisparityProcessor = 0;
#endif

} // initialize_providers

void MainModel_ColourSegmentationTracker::invalidate_providers() {
    m_DataProvider->invalidate();
    m_GrayscaleProvider->invalidate();
    m_Dense2DMotionProcessor->invalidate();
    if (m_DisparityProcessor) {
        m_DisparityProcessor->invalidate();
    }
} // invalidate_providers

void MainModel_ColourSegmentationTracker::deinitialize_providers() {
    delete m_Dense2DMotionProcessor; m_Dense2DMotionProcessor = 0;
    delete m_GrayscaleProvider; m_GrayscaleProvider = 0;
    if (m_DisparityProcessor) {
        delete m_DisparityProcessor;
        m_DisparityProcessor = 0;
    }
} // deinitialize_providers

void MainModel_ColourSegmentationTracker::initialize_filter(
    const FaceTrackerConfig & config) {

} // initialize_filter

void MainModel_ColourSegmentationTracker::deinitialize_filter() {

    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, m_Trackers) {
        delete tracker;
    }
    m_Trackers.clear();

} // deinitialize_filter

void MainModel_ColourSegmentationTracker::filter_detected_faces(
    const std::list<cvp_FaceDescriptor>& faces,
    std::list<cvp_FaceDescriptor>& init_candidates) {

    bicv_TrackerState mean_state;
    ip_RoiWindow mean_tracker_roi;
    bool consider_roi_flag;
    BOOST_FOREACH(cvp_FaceDescriptor fd, faces) {
        consider_roi_flag = true;
        BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, m_Trackers) {
            mean_state = mean(tracker->particle_distribution());
            mean_tracker_roi = HpParams2RoiConverter::params2roi(
                mean_state.m_ParamsCur);
            if (intersection_area(ip_RoiWindow::from_CvRect(fd.m_FaceRegion),
                    mean_tracker_roi) > 0) {
                tracker->observe_face(fd);
                consider_roi_flag = false;
            }
        }
        if (consider_roi_flag) {
            init_candidates.push_back(fd);
        }
    }
} // filter_detected_faces

BICV::bicv_ColourSegmentationTracker *
MainModel_ColourSegmentationTracker::try_initialize_head_pose_tracker(
    const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
    const boost::posix_time::ptime & timestamp) {

    // provide a separate instance of face colour model to each tracker!
    // it should be adapted while tracking
    FaceColorModel::FaceColorModel * tracker_colour_model =
        new FaceColorModel::FaceColorModel(m_Config.m_FaceColorModelConfig);
    IplImage * source_img = m_DataProvider->image();
    tracker_colour_model->prepare(source_img);

    // provide a separate instance of dynamic model to each tracker!
    bicv_CsMixtureDynamicModel * tracker_dynamic_model =
            new bicv_CsMixtureDynamicModel(m_Config.m_DynamicModelConfig,
            tracker_colour_model->pim_models_count());

    // initialize tracker with motion processor
    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    m_Dense2DMotionProcessor);

    bicv_ColourSegmentationTracker * tracker =
        new bicv_ColourSegmentationTracker(tracker_dynamic_model,
            m_DataProvider, motion_proc, tracker_colour_model,
            m_Config.m_NumTrackerParticles);
    tracker->init(init_candidate, timestamp);

    tracker->colour_model()->adapt_to(m_DataProvider->image(),
            init_candidate.m_FaceRegion);

    return tracker;

} // try_initialize_head_pose_tracker
