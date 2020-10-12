// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainModel_ColourSegmentationTracker - main model for the colour segmentation
//                                       tracking application
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MAINMODEL_COLOURSEGMENTATIONTRACKER_H__
#define __MAINMODEL_COLOURSEGMENTATIONTRACKER_H__

// SYSTEM INCLUDES
#include <list>                                     // STL list

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>                           // ROI
#include <image_processing/ip_Dense2DMotionProcessor.h>              // dense 2D motion processor
#include <bayes_image/bicv_GeneralDynamicModel.h>               // dynamic model
#include <bayes_image/bicv_ColourSegmentationTracker.h>         // tracker
#include <opencvplus/cvp_FaceDetectorCV.h>                      // OpenCV face detector
#include <opencvplus/FaceColorModel.h>                         // face color model
#include <opencvplus/cvp_FaceDetectorGroup.h>

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                    // config structure
#include "ColourSegmentationTrackerManager.h"       // tracker manager
#include "bi_ObjectObservation.h"

// SYSTEM INCLUDES
#include <boost/thread/shared_mutex.hpp>            // boost shared mutex
#include <boost/circular_buffer.hpp>                // boost circular buffer

#define LONGTERM_TRACKING

#ifdef LONGTERM_TRACKING
// forward declaration
class LongTermTrackerManager_ColourSegmentation;
#endif


/// @brief Main model for the colour segmentation tracking application
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class MainModel_ColourSegmentationTracker {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param data_provider Image provider using file/camera/image sequence
    /// @param face_detector_group Group of face detectors used by trackers
    /// @param config Structure containing all configuration options
    MainModel_ColourSegmentationTracker(
            ImageProcessing::ip_ImageProvider * data_provider,
            OpenCvPlus::cvp_FaceDetectorGroup * face_detector_group,
            const FaceTrackerConfig & config);

    /// Destructor
    ~MainModel_ColourSegmentationTracker();

    // OPERATIONS

    /// Update all providers and current states of all trackers
    void update();

    /// Operation used as a slot to communicate if any faces were observed.
    /// This operation is thread-safe, uses a mutex to queue occasional
    /// simultaneous calls.
    /// @param faces List of descriptors for detected faces
    void faces_observed(
            const std::list<OpenCvPlus::cvp_FaceDescriptor> & faces);

    /// Copies current face set to the provided buffer.
    /// This operation is thread-safe, uses a mutex to queue occasional
    /// simultaneous calls to faces_observed slot.
    /// @param faces List to copy face descriptors to
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

    /// Gives read access to local faces cache
    /// @return Observed faces cache
    const std::list<OpenCvPlus::cvp_FaceDescriptor> & faces_cache() const;

    ImageProcessing::ip_ImageProvider * data_provider() const {
        return m_DataProvider;
    }

    const std::list<BICV::bicv_ColourSegmentationTracker*>& trackers() const {
        return m_Trackers;
    }

    /// Tracker manager accumulated statistics on the tracker
    /// @param tracker Tracker for which to return accumulated statistics
    /// @return Statistics accumulated by tracker manager
    const BayesImage::bi_ObjectObservation * tracker_manager_observation(
            BICV::bicv_ColourSegmentationTracker * tracker) const;

    ImageProcessing::ip_Dense2DMotionProcessor *
    dense_motion_processor() const {
        return m_Dense2DMotionProcessor;
    }

    float performance_fps() const;

    FaceColorModel::FaceColorModel * face_colour_model() const {
        return m_GeneralFaceColorModel;
    }

    private:

    void initialize_providers(const FaceTrackerConfig & config);
    void initialize_filter(const FaceTrackerConfig & config);
    void invalidate_providers();
    void deinitialize_providers();
    void deinitialize_filter();

    void filter_detected_faces(
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces,
        std::list<OpenCvPlus::cvp_FaceDescriptor>& init_candidates);

    BICV::bicv_ColourSegmentationTracker * try_initialize_head_pose_tracker(
            const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
            const boost::posix_time::ptime & timestamp);

    // various data providers
    ImageProcessing::ip_ImageProvider * m_DataProvider;
    ImageProcessing::ip_ImageProvider * m_GrayscaleProvider;
    ImageProcessing::ip_ImageProvider * m_DisparityProcessor;

    // face detector group
    OpenCvPlus::cvp_FaceDetectorGroup * m_FaceDetectorGroup;

    std::list<BICV::bicv_ColourSegmentationTracker*> m_Trackers;

    // tracker manager to adjust tracker set - add/remove trackers
#ifdef LONGTERM_TRACKING
    LongTermTrackerManager_ColourSegmentation * m_TrackerManager;
#else
    ColourSegmentationTrackerManager * m_TrackerManager;
#endif

    FaceColorModel::FaceColorModel * m_GeneralFaceColorModel;

    // flag indicating that external face detector detected new faces
    volatile bool m_NewFacesDetected;
    // time associated to faces from an external face detector
    boost::posix_time::ptime m_NewFacesDetectedTime;
    // cache where external face detector stores detected faces
    std::list<OpenCvPlus::cvp_FaceDescriptor> m_FacesDetected;
    // faces considered during current iteration by the main model
    std::list<OpenCvPlus::cvp_FaceDescriptor> m_FacesCache;
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

    boost::circular_buffer<float> m_FpsValues;

    const FaceTrackerConfig & m_Config;

    // dense 2D motion processor
    ImageProcessing::ip_Dense2DMotionProcessor * m_Dense2DMotionProcessor;

};

#endif // __MAINMODEL_COLOURSEGMENTATIONTRACKER_H__
