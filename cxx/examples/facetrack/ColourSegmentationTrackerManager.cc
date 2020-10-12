// Copyright (c) 2011-2020 Idiap Research Institute
//
// ColourSegmentationTrackerManager - class to manage the set of colour
//                                    segmentation trackers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop

// LOCAL INCLUDES
#include "ColourSegmentationTrackerManager.h"

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BICV;

//////////////////////////////// PUBLIC //////////////////////////////////////

ColourSegmentationTrackerManager::ColourSegmentationTrackerManager(
        ip_ImageProvider * data_provider,
        const FaceTrackerConfig & config) :
            m_DataProvider(data_provider),
            m_Config(config) {
} // ColourSegmentationTrackerManager

ColourSegmentationTrackerManager::~ColourSegmentationTrackerManager() {
} // ~ColourSegmentationTrackerManager

void
ColourSegmentationTrackerManager::manage_trackers(
        const list<cvp_FaceDescriptor>& faces,
        const list<bicv_ColourSegmentationTracker*>& trackers,
        list<bicv_ColourSegmentationTracker*>& trackers_to_remove,
        list<ip_RoiWindow>& trackers_to_add) {

    // fill in the trackers to remove list
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers) {
        if (!should_keep_tracking(tracker, trackers)) {
            trackers_to_remove.push_back(tracker);
        }
    }

    // fill in the trackers to add list
    if (!faces.empty()) {
        trackers_to_add.push_back(ip_RoiWindow::from_CvRect(
            faces.front().m_FaceRegion));
    }

} // manage_trackers

//////////////////////////////// PRIVATE /////////////////////////////////////

bool
ColourSegmentationTrackerManager::should_keep_tracking(
        bicv_ColourSegmentationTracker* tracker,
        const list<bicv_ColourSegmentationTracker*>& trackers) {

    // the following is an example of code that uses the tracker's likelihood
    // model and feature producers to evaluate the likelihood

/*    ip_RoiWindow window;
    ip_HogFeatureProducerRounded * hfp = tracker->hog_feature_producer();
    const ip_HistogramTemplate& hog_feature = hfp->compute_feature(window);

    ip_SkinFeatureProducer * sfp = tracker->skin_feature_producer();
    const ip_SkinTemplate& skin_feature = sfp->compute_feature(window);

    bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);

    const bicv_HeadPoseTracker::Likelihood * lhood_model = tracker->likelihood();
    bicv_HeadPoseARTrackerState state;
    float lhood_value = lhood_model->evaluate(state, skinhog_obs);*/
    boost::posix_time::ptime tstamp = tracker->data_provider()->time();
    list<cvp_FaceDescriptor> fdlist =
            tracker->associated_face_detections_for_timestamp(tstamp);

    return !fdlist.empty() || tracker->should_continue_tracking();

} // should_keep_tracking
