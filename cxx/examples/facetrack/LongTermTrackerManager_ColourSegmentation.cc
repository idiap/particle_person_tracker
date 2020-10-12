// Copyright (c) 2011-2020 Idiap Research Institute
//
// LongTermTrackerManager - class to manage the set of trackers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
// Authors: Stefan Duffner
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop

// PROJECT INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>

// LOCAL INCLUDES
#include "LongTermTrackerManager_ColourSegmentation.h"

#include "image_processing/ip_TrackingMemory.h"
#include "image_processing/ip_CreateObject2.h"
#include "bayes_filter/bf_ConfidenceProb.h"
#include "bayes_filter/bi_ObjectObservation.h"

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BayesFilter;
using namespace BayesImage;
using namespace BICV;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

static const float OBJECT_OBSERVATION_UPDATE_RATE = 0.1;
static const float CONFIDENCE_LOWER_THRESHOLD = 0.01;
static const float STATE_INTERSECTION_UPPER_THRESHOLD = 0.95;
static const float TRACKING_MEMORY_UPDATE_FACTOR = 0.01;
static const float CREATE_OBJECT_UPDATE_FACTOR = 0.001;

static const float DEFAULT_FACE_DETECTOR_FREQUENCY = 0.0f;

//////////////////////////////// PUBLIC //////////////////////////////////////

LongTermTrackerManager_ColourSegmentation::LongTermTrackerManager_ColourSegmentation(
        ip_ImageProvider * data_provider, const FaceTrackerConfig & config) :
        m_DataProvider(data_provider), m_Config(config) {

    m_FaceColourModel = new FaceColorModel::FaceColorModel(
        m_Config.m_FaceColorModelConfig);

    m_iImageWidth = data_provider->image_buffer()->width;
    m_iImageHeight = data_provider->image_buffer()->height;
    m_pTrackingMemory = new ip_TrackingMemoryImage(data_provider,
            TRACKING_MEMORY_UPDATE_FACTOR);
    m_pCreateObject = new ip_CreateObject2(m_pTrackingMemory, m_iImageWidth,
            m_iImageHeight, CREATE_OBJECT_UPDATE_FACTOR);

    m_pCreateObject->init();
} // LongTermTrackerManager


LongTermTrackerManager_ColourSegmentation::~LongTermTrackerManager_ColourSegmentation() {
    delete m_pCreateObject;
    delete m_pTrackingMemory;
    delete m_FaceColourModel;
} // ~LongTermTrackerManager

ip_RoiWindow LongTermTrackerManager_ColourSegmentation::tRoi2tmRoi(
        const ip_RoiWindow& roi) {
    ip_RoiWindow rw(ip_RoiWindow::from_CvRect(
        m_FaceColourModel->fcm_roi2face_roi(ip_RoiWindow::to_CvRect(roi))));
    rw.m_iFirstColumn += 0.25 * rw.m_iWidth;
    rw.m_iFirstRow += 0.15 * rw.m_iHeight;
    rw.m_iWidth *= 0.5;
    rw.m_iHeight *= 0.7;
    return rw;
} // tRoi2tmRoi

void
LongTermTrackerManager_ColourSegmentation::manage_trackers(bool face_detections_reported,
	    const list<cvp_FaceDescriptor>& faces,
            const list<bicv_ColourSegmentationTracker*>& trackers,
            list<bicv_ColourSegmentationTracker*>& trackers_to_remove,
            list<cvp_FaceDescriptor>& trackers_to_add) {

    int cur_conf_thresh = 1.0;   // trivial threshold 1.0 on probability ratio
    list<cvp_FaceDescriptor> all_detections(faces.begin(), faces.end());

    list<ip_RoiWindow> tracker_mean_rois;
    list<cvp_FaceDescriptor> tracker_face_detections;

    // evaluate tracker mean ROIs, convert them to face detections
    mean_tracker_states_to_rois_and_face_descriptors(trackers,
            tracker_mean_rois, tracker_face_detections);
    all_detections.splice(all_detections.end(), tracker_face_detections);

    // update tracking memory with current mean state
    m_pTrackingMemory->update(tracker_mean_rois);

    int objnum = 1;
    bf_Confidence * conf;
    bi_ObjectObservation * oo;

  // fill in the trackers to remove list
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers) {

        // get or create confidence and observation
        get_or_create_tracker_data(tracker, conf, oo);

        if (face_detections_reported) {
            boost::posix_time::ptime current_data_time =
                tracker->data_provider()->time();
            list<cvp_FaceDescriptor> current_face_detections =
                tracker->associated_face_detections_for_timestamp(
                current_data_time);

            oo->m_bDetectionAssociated =
                    !current_face_detections.empty();
            oo->m_iIterSinceLastDetection = 0;
            oo->m_bUpdateDetection=true;
        } else {
            oo->m_bDetectionAssociated = false;
            oo->m_bUpdateDetection=false;
        }

        bicv_TrackerState mean_state = mean(tracker->particle_distribution());
        ip_RoiWindow roi = HpParams2RoiConverter::params2roi(mean_state.m_ParamsCur);
        int mx = int(roi.m_iFirstColumn + 0.5 * roi.m_iWidth);
        int my = int(roi.m_iFirstRow + 0.5 * roi.m_iHeight);
        if ((mx > 0) && (my > 0) && (mx < m_iImageWidth) && (my < m_iImageHeight)) {
            oo->m_fTrackingMemoryValue = m_pTrackingMemory->value(mx, my);
//            cout << "Tracker " << tracker << ", mem_value " << oo->m_fTrackingMemoryValue << endl;
        } else {
            oo->m_fTrackingMemoryValue = 0;
//            cout << "Tracker " << tracker << ", roi " << roi << ", zero mem value!" << endl;
        }

        bicv_TrackerState variance_state = variance(
                tracker->particle_distribution());
        variance_state = variance_state / mean_state.m_ParamsCur.m_Scale;
        oo->m_fCurVarianceNorm[0] = variance_state.m_ParamsCur.m_TranslationX;
        oo->m_fCurVarianceNorm[1] = variance_state.m_ParamsCur.m_TranslationY;
        oo->m_fCurVarianceNorm[2] = variance_state.m_ParamsCur.m_Scale;
        oo->m_fCurVarianceNorm[3] = variance_state.m_ParamsCur.m_Excentricity;

        float maxxyvar;
        maxxyvar = MAX(oo->m_fCurVarianceNorm[0], oo->m_fCurVarianceNorm[1]);

        if (oo->m_bFirstObservation) {
            oo->m_fRACurVariance = maxxyvar;
            oo->m_fRACurVariance2 = maxxyvar * maxxyvar;
        } else {
            oo->m_fRACurVariance  =
                (1.0 - OBJECT_OBSERVATION_UPDATE_RATE) * oo->m_fRACurVariance +
                OBJECT_OBSERVATION_UPDATE_RATE * maxxyvar;
            oo->m_fRACurVariance2 =
                (1.0 - OBJECT_OBSERVATION_UPDATE_RATE) * oo->m_fRACurVariance2 +
                OBJECT_OBSERVATION_UPDATE_RATE * maxxyvar * maxxyvar;
        }
        oo->m_fRVCurVariance = oo->m_fRACurVariance2 -
                oo->m_fRACurVariance * oo->m_fRACurVariance;
        oo->m_fHTM_Variance = oo->m_fHTM_Variance + maxxyvar -
                oo->m_fRACurVariance + 0.5 * sqrt(oo->m_fRVCurVariance);
        oo->m_fHTU_Variance = oo->m_fHTU_Variance + maxxyvar -
                oo->m_fRACurVariance - 0.5 * sqrt(oo->m_fRVCurVariance);
        if (oo->m_fHTU_Variance<oo->m_fHTUmin_Variance) {
            oo->m_fHTUmin_Variance = oo->m_fHTU_Variance;
        }
        if (oo->m_fHTM_Variance>oo->m_fHTMmax_Variance) {
            oo->m_fHTMmax_Variance = oo->m_fHTM_Variance;
        }

        const bicv_ColourSegmentationTracker::Likelihood * lhood_model =
            tracker->likelihood();
        float lhood_value = lhood_model->evaluate(mean_state,
                m_DataProvider->image());
        oo->m_fLikelihoodMean = lhood_value;
//        cout << "Tracker " << tracker << ", lhood_value " << lhood_value << endl;

        if (oo->m_bFirstObservation) {
            oo->m_fRALikelihoodMean = lhood_value;
            oo->m_fRALikelihoodMean2 = lhood_value * lhood_value;
        } else {
            oo->m_fRALikelihoodMean =
                (1.0 - OBJECT_OBSERVATION_UPDATE_RATE) * oo->m_fRALikelihoodMean +
                OBJECT_OBSERVATION_UPDATE_RATE * lhood_value;
            oo->m_fRALikelihoodMean2 =
                (1.0 - OBJECT_OBSERVATION_UPDATE_RATE) * oo->m_fRALikelihoodMean2 +
                OBJECT_OBSERVATION_UPDATE_RATE * lhood_value*lhood_value;
        }
        oo->m_fRVLikelihoodMean = oo->m_fRALikelihoodMean2 -
                oo->m_fRALikelihoodMean * oo->m_fRALikelihoodMean;
        oo->m_fHTM_Likelihood = oo->m_fHTM_Likelihood + lhood_value -
                oo->m_fRALikelihoodMean + 0.5 * sqrt(oo->m_fRVLikelihoodMean);
        if (oo->m_fHTM_Likelihood>oo->m_fHTMmax_Likelihood) {
            oo->m_fHTMmax_Likelihood = oo->m_fHTM_Likelihood;
        }
        oo->m_iIterSinceLastDetection++;

        //printf("%d tm: %f  lh: %.9f  lh_hog: %.9f  lh_skin: %.9f  var: %f\n", objnum, oo->m_fTrackingMemoryValue, oo->m_fLikelihoodMean, lhood_hog, lhood_skin, maxxyvar);

        conf->update(face_detections_reported, oo);
        //printf("   conf: %f\n", conf->getValue());

        if (conf->getValue() < CONFIDENCE_LOWER_THRESHOLD) {
            trackers_to_remove.push_back(tracker);
            m_ConfidenceMap.erase(tracker);
            m_ObservationMap.erase(tracker);
        } else {
            // check if the bounding box coincides  with any of other trackers:
            BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker_other, trackers) {
                // tracker should be different from the currently considered one
                // and should not be already deleted
                if ((tracker_other != tracker) &&
                    (trackers_to_remove.end() == find(trackers_to_remove.begin(), trackers_to_remove.end(), tracker_other))) {
                    bicv_TrackerState mean_state_other = mean(tracker_other->particle_distribution());
                    ip_RoiWindow roi_other = HpParams2RoiConverter::params2roi(mean_state_other.m_ParamsCur);
    //                cout << "intersection area " << intersection_area(roi, roi_other) << endl;
                    if (static_cast<float>(intersection_area(roi, roi_other)) / area(roi) > STATE_INTERSECTION_UPPER_THRESHOLD) {
    //                    cout << "intersection area rate " << static_cast<float>(intersection_area(roi, roi_other)) / area(roi) << endl;
                        // keep tracker with lower ID
                        if (tracker->id() < tracker_other->id()) {
                            trackers_to_remove.push_back(tracker_other);
                            m_ConfidenceMap.erase(tracker_other);
                            m_ObservationMap.erase(tracker_other);
                        } else if (trackers_to_remove.end() == find(trackers_to_remove.begin(), trackers_to_remove.end(), tracker)) {
                            trackers_to_remove.push_back(tracker);
                            m_ConfidenceMap.erase(tracker);
                            m_ObservationMap.erase(tracker);
                        }
                    }
                }
            }
        }
        objnum++;
    }
    //printf("\n");

    // reset object observations
    map<bicv_ColourSegmentationTracker*, bi_ObjectObservation*>::iterator oit;
    for(oit=m_ObservationMap.begin(); oit!=m_ObservationMap.end(); ++oit) {
        oit->second->reset();
        oit->second->m_bFirstObservation=false;
    }

    m_pCreateObject->update(face_detections_reported, all_detections);
    if (face_detections_reported) {
        //m_pCreateObject->save("tmp.png");
        // fill in the trackers to add list
        BOOST_FOREACH(cvp_FaceDescriptor f, faces) {
            ip_RoiWindow rw = ip_RoiWindow::from_CvRect(f.m_FaceRegion);
            float mean_co_val = 0.33333 * (
                    m_pCreateObject->getValue(
                            rw.m_iFirstColumn + 0.5 * rw.m_iWidth,
                            rw.m_iFirstRow + 0.5 * rw.m_iHeight)
                    + m_pCreateObject->getValue(
                            rw.m_iFirstColumn + 0.25 * rw.m_iWidth,
                            rw.m_iFirstRow + 0.25 * rw.m_iHeight)
                    + m_pCreateObject->getValue(
                            rw.m_iFirstColumn + 0.75 * rw.m_iWidth,
                            rw.m_iFirstRow + 0.75 * rw.m_iHeight));
//            cout << "Check " << rw << ", mem_value " << mean_co_val << endl;
            //printf("mean_co_val: %f\n", mean_co_val);
            if (mean_co_val > cur_conf_thresh) {
              trackers_to_add.push_back(f);
            }
        }
    }

} // manage_trackers

const BayesImage::bi_ObjectObservation *
LongTermTrackerManager_ColourSegmentation::get_observation(
        BICV::bicv_ColourSegmentationTracker * tracker) {
    map<bicv_ColourSegmentationTracker*, bi_ObjectObservation*>::iterator it =
            m_ObservationMap.find(tracker);
    if (it == m_ObservationMap.end()) {
        return 0;
    } else {
        return it->second;
    }
} // get_observation

//////////////////////////////// PRIVATE /////////////////////////////////////

void LongTermTrackerManager_ColourSegmentation::
    mean_tracker_states_to_rois_and_face_descriptors(
        const list<bicv_ColourSegmentationTracker*>& trackers,
        list<ip_RoiWindow>& rois,
        list<cvp_FaceDescriptor>& face_descriptors) {

    cvp_FaceDescriptor fdtmp;
    fdtmp.m_DetectionTime = m_DataProvider->time();
    fdtmp.m_ImageHeight = m_iImageHeight;
    fdtmp.m_ImageWidth = m_iImageWidth;

    bicv_TrackerState mean_state;
    ip_RoiWindow mean_roi;
    float mean_pan_angle;
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers) {
        // keep tracker mean ROI to update tracking memory
        mean_state = mean(tracker->particle_distribution());
        mean_roi = HpParams2RoiConverter::params2roi(mean_state.m_ParamsCur);
        rois.push_back(mean_roi);
        // add tracker state to the list of detected faces
        fdtmp.m_FaceRegion = ip_RoiWindow::to_CvRect(mean_roi);
        fdtmp.m_Pose = CVP_FACEDETECTOR_FACE;
        face_descriptors.push_back(fdtmp);
    }

} // mean_tracker_states_to_rois_and_face_descriptors

void LongTermTrackerManager_ColourSegmentation::get_or_create_tracker_data(
        bicv_ColourSegmentationTracker * tracker,
        BayesFilter::bf_Confidence *& confidence,
        BayesImage::bi_ObjectObservation *& observation) {

    map<bicv_ColourSegmentationTracker*, bf_Confidence*>::iterator it;
    it = m_ConfidenceMap.find(tracker);
    if (it == m_ConfidenceMap.end()) {
        confidence = new bf_ConfidenceProb(DEFAULT_FACE_DETECTOR_FREQUENCY);
        confidence->reset();
        observation = new bi_ObjectObservation();
        observation->reset();
        observation->resetHinckleyTestVariables();
        observation->m_bFirstObservation = true;
        m_ConfidenceMap.insert(
                pair<bicv_ColourSegmentationTracker*, bf_Confidence*>(
                tracker, confidence));
        m_ObservationMap.insert(
                pair<bicv_ColourSegmentationTracker*,bi_ObjectObservation*>(
                tracker, observation));
    } else {
        confidence = it->second;
        observation = m_ObservationMap[tracker];
    }
} // get_or_create_tracker_data

/*
bool
LongTermTrackerManager::should_keep_tracking(bicv_HeadPoseTracker* tracker,
        const list<bicv_HeadPoseTracker*>& trackers) {
	*/

    // the following is an example of code that uses the tracker's likelihood
    // model and feature producers to evaluate the likelihood

/*    ip_RoiWindow window;
    ip_HogFeatureProducerRounded * hfp = tracker->hog_feature_producer();
    const ip_HistogramTemplate& hog_feature = hfp->compute_feature(window);

    ip_SkinFeatureProducer * sfp = tracker->skin_feature_producer();
    const ip_SkinTemplate& skin_feature = sfp->compute_feature(window);

    bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);

    const bicv_HeadPoseTracker::Likelihood * lhood_model = tracker->likelihood();
    bicv_HeadPoseARTrackerState state;  // needs to be set
    float lhood_value = lhood_model->evaluate(state, skinhog_obs);*/

/*
    return true; //tracker->face_observed() || tracker->should_continue_tracking();

} // should_keep_tracking
*/
