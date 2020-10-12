// Copyright (c) 2011-2020 Idiap Research Institute
//
// ColourSegmentationTrackerManager - class to manage the set of colour
//                                    segmentation trackers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __COLOURSEGMENTATIONTRACKERMANAGER_H__
#define __COLOURSEGMENTATIONTRACKERMANAGER_H__

// SYSTEM INCLUDES
#include <list>                                     // STL list

// PROJECT INCLUDES
#include <image_processing/ip_ImageProvider.h>          // image provider
#include <image_processing/ip_RoiWindow.h>              // ROI
#include <bayes_image/bicv_ColourSegmentationTracker.h> // head pose tracker

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                    // config structure

/// @brief Manager for the set of trackers
///
/// Initializes new trackers when appropriate and removes the old ones that
/// cannot track any more.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    10.11.2011

class ColourSegmentationTrackerManager {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param data_provider Source image provider
    /// @param config Configuration options structure
    ColourSegmentationTrackerManager(
            ImageProcessing::ip_ImageProvider * data_provider,
            const FaceTrackerConfig & config);

    /// Destructor
    ~ColourSegmentationTrackerManager();

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
            const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces,
            const std::list<BICV::bicv_ColourSegmentationTracker*>& trackers,
            std::list<BICV::bicv_ColourSegmentationTracker*>& trackers_to_remove,
            std::list<ImageProcessing::ip_RoiWindow>& trackers_to_add);

    private:

    // Returns true if the tracker can continue tracking, false otherwise
    // Used to split the trackers set into active trackers
    // (that should keep on tracking) and trackers that should be removed
    bool should_keep_tracking(BICV::bicv_ColourSegmentationTracker* tracker,
            const std::list<BICV::bicv_ColourSegmentationTracker*>& trackers);

    // source image provider
    ImageProcessing::ip_ImageProvider * m_DataProvider;
    // general config options
    const FaceTrackerConfig & m_Config;

};

#endif // __COLOURSEGMENTATIONTRACKERMANAGER_H__
