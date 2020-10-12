// Copyright (c) 2011-2020 Idiap Research Institute
//
// LongTermTrackerManager - class to manage the set of trackers
// LongTermTrackerManager.cc
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
// Authors: Stefan Duffner
//
// See COPYING file for the complete license text.

#ifndef __LONGTERMTRACKERMANAGER_H__
#define __LONGTERMTRACKERMANAGER_H__

// SYSTEM INCLUDES
#include <list>                                     // STL list
#include <map>

// PROJECT INCLUDES
#include <image_processing/ip_ImageProvider.h>          // image provider
#include <image_processing/ip_RoiWindow.h>              // ROI
#include <image_processing/ip_TrackingMemoryImage.h>    // tracking memory

#include <image_processing/ip_BoundingBox.h>
#include <bayes_image/bicv_HeadPoseTracker.h>                   // head pose tracker

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                    // config structure

namespace ImageProcessing
{
  class ip_TrackingMemory;
  class ip_CreateObject2;
}

namespace BayesFilter
{
  class bf_Confidence;
}

namespace BayesImage
{
  class bi_ObjectObservation;
}

/// @brief Manager for the set of trackers
///
/// Initialises new trackers when appropriate and removes the old ones that
/// cannot track any more.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    04.11.2011

class LongTermTrackerManager {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param data_provider Source image provider
    /// @param config Configuration options structure
    LongTermTrackerManager(ImageProcessing::ip_ImageProvider * data_provider,
            const FaceTrackerConfig & config);

    /// Destructor
    ~LongTermTrackerManager();

    // OPERATIONS

    /// Method that uses face detections that are not associated to any
    /// of the trackers to update the set of trackers - create new ones and/or
    /// remove old ones
    /// @param faces List of descriptors for detected faces that are
    /// not associated to any of the current trackers
    /// @param trackers List of current trackers
    /// @param trackers_to_remove List of current trackers to remove
    /// @param trackers_to_add List of trackers to add based on image ROIs
    void manage_trackers(
            bool face_detections_reported,
            const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces,
            const std::list<BICV::bicv_HeadPoseTracker*>& trackers,
            std::list<BICV::bicv_HeadPoseTracker*>& trackers_to_remove,
            std::list<OpenCvPlus::cvp_FaceDescriptor>& trackers_to_add);

    /// Method that fills in collected statistics for a given tracker.
    /// @param tracker Tracker for which to return the statistics
    /// @return
    const BayesImage::bi_ObjectObservation * get_observation(
            BICV::bicv_HeadPoseTracker * tracker);

    private:

    /// Converts tracker states to regions of interest by taking the mean of
    /// tracker particles.
    /// @param trackers Trackers to compute means for
    /// @param rois List of regions of interest to fill
    /// @param face_descriptors List of face descriptors to fill
    void mean_tracker_states_to_rois_and_face_descriptors(
            const std::list<BICV::bicv_HeadPoseTracker*>& trackers,
            std::list<ImageProcessing::ip_RoiWindow>& rois,
            std::list<OpenCvPlus::cvp_FaceDescriptor>& face_descriptors);

    /// Tries to obtain data corresponding to the tracker from confidence and
    /// observation maps. If data does not exist, creates it.
    /// @param tracker Tracker to obtain data for
    /// @param confidence Confidence data associated with the tracker
    /// @param observation Observation data associated with the tracker
    void get_or_create_tracker_data(
            BICV::bicv_HeadPoseTracker * tracker,
            BayesFilter::bf_Confidence *& confidence,
            BayesImage::bi_ObjectObservation *& observation);

    // Returns true if the tracker can continue tracking, false otherwise
    // Used to split the trackers set into active trackers
    // (that should keep on tracking) and trackers that should be removed
    /*
    bool should_keep_tracking(BICV::bicv_HeadPoseTracker* tracker,
            const std::list<BICV::bicv_HeadPoseTracker*>& trackers);
	    */

    // data provider image width, cached value
    int m_iImageWidth;
    // data provider image height, cached value
    int m_iImageHeight;

    // source image provider
    ImageProcessing::ip_ImageProvider * m_DataProvider;
    // general config options
    const FaceTrackerConfig & m_Config;

    ImageProcessing::ip_TrackingMemoryImage * m_pTrackingMemory;
    ImageProcessing::ip_CreateObject2* m_pCreateObject;
    /// maps trackers to their confidence map
    std::map<BICV::bicv_HeadPoseTracker*, BayesFilter::bf_Confidence*>
        m_ConfidenceMap;
    /// maps trackers to their long-term observations
    std::map<BICV::bicv_HeadPoseTracker*,BayesImage::bi_ObjectObservation*>
        m_ObservationMap;
};

#endif // __LONGTERMTRACKERMANAGER_H__
