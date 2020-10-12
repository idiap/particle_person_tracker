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
#include <bayes_image/bicv_HeadPoseLikelihoodModel.h>

// LOCAL INCLUDES
#include "LongTermTrackerManager.h"

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

static const float TRACKING_MEMORY_UPDATE_FACTOR = 0.01f;
static const float HEAD_POSE_FACE_PROFILE_THREHSOLD = 30.0f;

static const float DEFAULT_FACE_DETECTOR_FREQUENCY = 0.0f;

//////////////////////////////// PUBLIC //////////////////////////////////////

LongTermTrackerManager::
LongTermTrackerManager(ip_ImageProvider * data_provider,
                       const FaceTrackerConfig & config):
  m_DataProvider(data_provider),
  m_Config(config)
{
  m_iImageWidth  = data_provider->image_buffer()->width;
  m_iImageHeight = data_provider->image_buffer()->height;

  m_pTrackingMemory =
    new ip_TrackingMemoryImage(data_provider,
                               TRACKING_MEMORY_UPDATE_FACTOR);

  m_pCreateObject =
    new ip_CreateObject2(m_pTrackingMemory,
                         m_iImageWidth,
                         m_iImageHeight,
                         0.001);

  m_pCreateObject->init();

  std::cout << "[LongTermTrackerManager] MinFaceHeightRatio "
            << m_Config.m_MinFaceHeightRatio << std::endl;
} // LongTermTrackerManager


LongTermTrackerManager::
~LongTermTrackerManager()
{
  delete m_pCreateObject;
  delete m_pTrackingMemory;
  // TODO: delete the contents of confidence and observation maps
}

void
LongTermTrackerManager::
manage_trackers(bool face_detections_reported,
                const list<cvp_FaceDescriptor>& faces,
                const list<bicv_HeadPoseTracker*>& trackers,
                list<bicv_HeadPoseTracker*>& trackers_to_remove,
                list<cvp_FaceDescriptor>& trackers_to_add)
{
  int cur_conf_thresh = 1.0;   // trivial threshold 1.0 on probability ratio
  list<cvp_FaceDescriptor> all_detections(faces.begin(), faces.end());

  list<ip_RoiWindow> tracker_mean_rois;
  list<cvp_FaceDescriptor> tracker_face_detections;

  // evaluate tracker mean ROIs, convert them to face detections
  mean_tracker_states_to_rois_and_face_descriptors(trackers,
                                                   tracker_mean_rois,
                                                   tracker_face_detections);
  all_detections.splice(all_detections.end(), tracker_face_detections);

  // update tracking memory with current tracker mean states
  //  cout << "Start update tracker memory..." << endl;
  m_pTrackingMemory->update(tracker_mean_rois);
  //  cout << "End update tracker memory..." << endl;

  int objnum = 1;
  bf_Confidence * confidence;
  bi_ObjectObservation * observation;

  // fill in the trackers to remove list
  BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers) {

    // get or create confidence and observation
    get_or_create_tracker_data(tracker, confidence, observation);

    boost::posix_time::ptime current_data_time =
      tracker->data_provider()->time();

    // std::cout << "current_data_time " << current_data_time << std::endl;
    // std::cout << "observation->m_TimeLastDetection " << observation->m_TimeLastDetection << std::endl;

    if (face_detections_reported)
      {
        list<cvp_FaceDescriptor> current_face_detections =
          tracker->associated_face_detections_for_timestamp(current_data_time);

        observation->m_bDetectionAssociated = !current_face_detections.empty();

        // Originally, m_iIterSinceLastDetection is always reset to 0,
        // even when no detection is found for the tracker. Now we
        // changed that and reset the value only when a detection is
        // actually found by the tracker. This allows the
        // bf_ConfidenceProb to have a different behaviour when no
        // detection has been made for a long time.
        if (observation->m_bDetectionAssociated)
          {
            observation->m_iIterSinceLastDetection = 0;
            observation->m_TimeLastDetection = current_data_time;
          }

        observation->m_bUpdateDetection = true;
      }
    else
      {
        observation->m_bDetectionAssociated = false;
        observation->m_bUpdateDetection = false;
      }

    bicv_HeadPoseARTrackerState mean_state =
      mean(tracker->particle_distribution());

    ip_RoiWindow roi =
      HpParams2RoiConverter::hpparams2roi(mean_state.m_HeadPoseParamsCur);

    const double bb_height_ratio = static_cast<double>(roi.m_iHeight)/static_cast<double>(m_iImageHeight);

    // std::cout << "W " << roi.m_iWidth
    //           << " H " << roi.m_iHeight
    //           << " H% " << static_cast<double>(roi.m_iHeight)/static_cast<double>(m_iImageHeight)
    //           << " height " << m_iImageHeight
    //           << std::endl;

    int mx, my;
    mx = int(roi.m_iFirstColumn + 0.5 * roi.m_iWidth);
    my = int(roi.m_iFirstRow + 0.5 * roi.m_iHeight);
    if (mx>0 && my>0 && mx<m_iImageWidth && my<m_iImageHeight)
      {
        observation->m_fTrackingMemoryValue = m_pTrackingMemory->value
          (int(roi.m_iFirstColumn + 0.5 * roi.m_iWidth),
           int(roi.m_iFirstRow + 0.5 * roi.m_iHeight));
    }
    else
      {
        observation->m_fTrackingMemoryValue = 0;
      }

    bicv_HeadPoseARTrackerState variance_state =
      variance(tracker->particle_distribution());
    variance_state = variance_state/mean_state.m_HeadPoseParamsCur.m_Scale;
    observation->m_fCurVarianceNorm[0] =
      variance_state.m_HeadPoseParamsCur.m_TranslationX;
    observation->m_fCurVarianceNorm[1] =
      variance_state.m_HeadPoseParamsCur.m_TranslationY;
    observation->m_fCurVarianceNorm[2] =
      variance_state.m_HeadPoseParamsCur.m_Scale;
    observation->m_fCurVarianceNorm[3] =
      variance_state.m_HeadPoseParamsCur.m_Excentricity;

    float maxxyvar;
    maxxyvar = MAX(observation->m_fCurVarianceNorm[0],
                   observation->m_fCurVarianceNorm[1]);

    if (observation->m_bFirstObservation) {
      observation->m_fRACurVariance = maxxyvar;
      observation->m_fRACurVariance2 = maxxyvar * maxxyvar;
    } else {
      observation->m_fRACurVariance =
        0.9 * observation->m_fRACurVariance + 0.1 * maxxyvar;
      observation->m_fRACurVariance2 =
        0.9 * observation->m_fRACurVariance2 +
        0.1 * maxxyvar * maxxyvar;
    }

    observation->m_fRVCurVariance = observation->m_fRACurVariance2 -
      observation->m_fRACurVariance * observation->m_fRACurVariance;
    observation->m_fHTM_Variance = observation->m_fHTM_Variance + maxxyvar -
      observation->m_fRACurVariance +
      0.5 * sqrt(observation->m_fRVCurVariance);
    observation->m_fHTU_Variance = observation->m_fHTU_Variance + maxxyvar -
      observation->m_fRACurVariance -
      0.5 * sqrt(observation->m_fRVCurVariance);
    if (observation->m_fHTU_Variance < observation->m_fHTUmin_Variance) {
      observation->m_fHTUmin_Variance = observation->m_fHTU_Variance;
    }
    if (observation->m_fHTM_Variance > observation->m_fHTMmax_Variance) {
      observation->m_fHTMmax_Variance = observation->m_fHTM_Variance;
    }

    //    cout << "Start evaluate likelihood..." << endl;
    ip_HogFeatureProducerRounded * hfp = tracker->hog_feature_producer();
    const ip_HistogramTemplate& hog_feature = hfp->compute_feature(roi);

    ip_SkinFeatureProducer * sfp = tracker->skin_feature_producer();
    const ip_SkinTemplate& skin_feature = sfp->compute_feature(roi);

    bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);

    const bicv_HeadPoseTracker::Likelihood * lhood_model =
      tracker->likelihood();

    const bicv_HeadPoseLikelihoodModel* hplm =
      dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(lhood_model);

    float lhood_hog, lhood_skin;
    if (hplm)
      {
        lhood_hog = hplm->evaluate_HoG(mean_state, skinhog_obs);
        lhood_skin = hplm->evaluate_Skin(mean_state, skinhog_obs);
      }
    float lhood_value = lhood_model->evaluate(mean_state, skinhog_obs);
    observation->m_fLikelihoodMean = lhood_value;

    if (observation->m_bFirstObservation) {
      observation->m_fRALikelihoodMean = lhood_value;
      observation->m_fRALikelihoodMean2 = lhood_value*lhood_value;
    } else {
      observation->m_fRALikelihoodMean =
        0.9*observation->m_fRALikelihoodMean + 0.1*lhood_value;
      observation->m_fRALikelihoodMean2 =
        0.9*observation->m_fRALikelihoodMean2 + 0.1*lhood_value*lhood_value;
    }
    observation->m_fRVLikelihoodMean = observation->m_fRALikelihoodMean2 -
      observation->m_fRALikelihoodMean * observation->m_fRALikelihoodMean;
    observation->m_fHTM_Likelihood = observation->m_fHTM_Likelihood +
      lhood_value - observation->m_fRALikelihoodMean +
      0.5 * sqrt(observation->m_fRVLikelihoodMean);
    if (observation->m_fHTM_Likelihood > observation->m_fHTMmax_Likelihood) {
      observation->m_fHTMmax_Likelihood = observation->m_fHTM_Likelihood;
    }

    observation->m_iIterSinceLastDetection++;

    confidence->update(observation->m_bDetectionAssociated, observation);

    boost::posix_time::time_duration time_since_first_detection =
      current_data_time - observation->m_TimeLastDetection;


    // std::cout << "id() " << tracker->id()
    //           << " is_asso " << observation->m_bDetectionAssociated
    //           << " confidence " << confidence->getValue()
    //           << " last " << observation->m_iIterSinceLastDetection
    //           << " since " << time_since_first_detection.total_milliseconds()
    //           << std::endl;

    // TODO: Consider m_iIterSinceLastDetection in a better way
    //  || observation->m_iIterSinceLastDetection > 5
    // if (confidence->getValue()<0.01 || observation->m_iIterSinceLastDetection > 5)
    //
    // DONT'T USE observation->m_iIterSinceLastDetection > SOMETHING
    // because this depends on the speed of the detector, frame rate,
    // image size, etc.

    // ORIGINAL condition only based on confidence
    // if (confidence->getValue()<0.01)

    // std::cout << "id " << tracker->id() << " " << bb_height_ratio << std::endl;

    if (confidence->getValue()<0.01 ||
        time_since_first_detection.total_milliseconds() > 2000 ||
        bb_height_ratio < m_Config.m_MinFaceHeightRatio)
      {
        // std::cout << "bb_height_ratio " << bb_height_ratio << std::endl;
        // std::cout << "Remove tracklet conf " << confidence->getValue()
        //           << " time since last "
        //           << time_since_first_detection.total_milliseconds()
        //           << std::endl;
        // std::cout << "Remove tracker " << tracker->id() << std::endl;
        trackers_to_remove.push_back(tracker);
        delete confidence;
        delete observation;
        m_ConfidenceMap.erase(tracker);
        m_ObservationMap.erase(tracker);
      }
    else
      {
        // Check if the bounding box coincides with any of other trackers:
        BOOST_FOREACH(bicv_HeadPoseTracker * tracker_other, trackers) {
          // tracker should be different from the currently considered one
          // and should not be already deleted
          if ((tracker_other != tracker) &&
              (trackers_to_remove.end() == find(trackers_to_remove.begin(),
                                                trackers_to_remove.end(),
                                                tracker_other)))
            {
              bicv_HeadPoseARTrackerState mean_state_other =
                mean(tracker_other->particle_distribution());

              ip_RoiWindow roi_other = HpParams2RoiConverter::
                hpparams2roi(mean_state_other.m_HeadPoseParamsCur);

              // float intersection =
              //   static_cast<float>(intersection_area(roi, roi_other)) /
              //   area(roi);

              float intersection = static_cast<float>(intersection_area(roi, roi_other)) /
                static_cast<float>(union_area(roi, roi_other));

              // std::cout << "trackers " << tracker->id() << "/" << tracker_other->id()
              //           << " intersection " << intersection
              //           << std::endl;

              if (intersection > m_Config.m_RoiOverlapThreshold)
                {
                  // std::cout << "Remove because overlap" << std::endl;
                  // Remove both
                  trackers_to_remove.push_back(tracker_other);
                  m_ConfidenceMap.erase(tracker_other);
                  m_ObservationMap.erase(tracker_other);
                  trackers_to_remove.push_back(tracker);
                  m_ConfidenceMap.erase(tracker);
                  m_ObservationMap.erase(tracker);

                  // // keep tracker with lower ID
                  // if (tracker->id() < tracker_other->id())
                  //   {
                  //     trackers_to_remove.push_back(tracker_other);
                  //     m_ConfidenceMap.erase(tracker_other);
                  //     m_ObservationMap.erase(tracker_other);
                  //   }
                  // else if (trackers_to_remove.end() ==
                  //          find(trackers_to_remove.begin(),
                  //               trackers_to_remove.end(),
                  //               tracker))
                  //   {
                  //     trackers_to_remove.push_back(tracker);
                  //     m_ConfidenceMap.erase(tracker);
                  //     m_ObservationMap.erase(tracker);
                  //   }
                }
            }
        } // for tracker_other
      }
    objnum++;
  } // for tracker
  //printf("\n");

  // reset object observations
  map<bicv_HeadPoseTracker*,bi_ObjectObservation*>::iterator oit;
  for(oit=m_ObservationMap.begin(); oit!=m_ObservationMap.end(); ++oit)
    {
      oit->second->reset();
      oit->second->m_bFirstObservation=false;
    }

  //  cout << "Start CO update..." << endl;
  m_pCreateObject->update(face_detections_reported, all_detections);
  //  cout << "End CO update..." << endl;
  if (face_detections_reported)
    {
      //m_pCreateObject->save("tmp.png");
      // fill in the trackers to add list
      BOOST_FOREACH(cvp_FaceDescriptor f, faces)
        {
          ip_RoiWindow rw = ip_RoiWindow::from_CvRect(f.m_FaceRegion);

          const double face_height_ratio = static_cast<double>(rw.m_iHeight)/static_cast<double>(m_iImageHeight);

          if(face_height_ratio < m_Config.m_MinFaceHeightRatio) continue;

          float mean_co_val = 0.33333*
            (m_pCreateObject->getValue(rw.m_iFirstColumn + 0.5*rw.m_iWidth,
                                       rw.m_iFirstRow    + 0.5*rw.m_iHeight)
             + m_pCreateObject->getValue(rw.m_iFirstColumn + 0.25*rw.m_iWidth,
                                         rw.m_iFirstRow    + 0.25*rw.m_iHeight)
             + m_pCreateObject->getValue(rw.m_iFirstColumn + 0.75*rw.m_iWidth,
                                         rw.m_iFirstRow    + 0.75*rw.m_iHeight));
          // printf("mean_co_val: %f\n", mean_co_val);
          if (mean_co_val>cur_conf_thresh)
            {
              trackers_to_add.push_back(f);
            }
        }
    }

  //delete [] curMeanBB;
  /*
    if (!faces.empty()) {
    trackers_to_add.push_back(faces.front().m_FaceRegion);
    }
  */

} // manage_trackers

const BayesImage::bi_ObjectObservation*
LongTermTrackerManager::get_observation(
        BICV::bicv_HeadPoseTracker * tracker) {
    map<bicv_HeadPoseTracker*, bi_ObjectObservation*>::iterator it =
            m_ObservationMap.find(tracker);
    if (it == m_ObservationMap.end()) {
        return 0;
    } else {
        return it->second;
    }
} // get_tracker_data

//////////////////////////////// PRIVATE /////////////////////////////////////

void LongTermTrackerManager::mean_tracker_states_to_rois_and_face_descriptors(
        const list<bicv_HeadPoseTracker*>& trackers,
        list<ip_RoiWindow>& rois,
        list<cvp_FaceDescriptor>& face_descriptors) {

    cvp_FaceDescriptor fdtmp;
    fdtmp.m_DetectionTime = m_DataProvider->time();
    fdtmp.m_ImageHeight = m_iImageHeight;
    fdtmp.m_ImageWidth = m_iImageWidth;

    bicv_HeadPoseARTrackerState mean_state;
    ip_RoiWindow mean_roi;
    float mean_pan_angle;
    BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers) {
        // keep tracker mean ROI to update tracking memory
        mean_state = mean(tracker->particle_distribution());
        mean_roi = HpParams2RoiConverter::hpparams2roi(
                mean_state.m_HeadPoseParamsCur);
        mean_pan_angle = mean_state.m_HeadPoseParamsCur.m_HeadPose.pan();
        rois.push_back(mean_roi);
        // add tracker state to the list of detected faces
        fdtmp.m_FaceRegion = ip_RoiWindow::to_CvRect(mean_roi);
        fdtmp.m_Pose = (fabs(mean_pan_angle) < HEAD_POSE_FACE_PROFILE_THREHSOLD) ?
                CVP_FACEDETECTOR_FACE :
                ((mean_pan_angle > 0) ?
                        CVP_FACEDETECTOR_PROFILE_LEFT :
                        CVP_FACEDETECTOR_PROFILE_RIGHT);
        face_descriptors.push_back(fdtmp);
    }

} // mean_tracker_states_to_rois_and_face_descriptors

void LongTermTrackerManager::get_or_create_tracker_data(
        BICV::bicv_HeadPoseTracker * tracker,
        BayesFilter::bf_Confidence *& confidence,
        BayesImage::bi_ObjectObservation *& observation) {

    map<bicv_HeadPoseTracker*, bf_Confidence*>::iterator it;
    it = m_ConfidenceMap.find(tracker);
    if (it == m_ConfidenceMap.end()) {
        confidence = new bf_ConfidenceProb(DEFAULT_FACE_DETECTOR_FREQUENCY);
        confidence->reset();
        observation = new bi_ObjectObservation();
        observation->reset();
        observation->resetHinckleyTestVariables();
        observation->m_bFirstObservation = true;
        observation->m_TimeLastDetection = m_DataProvider->time();
        m_ConfidenceMap.insert(
                pair<bicv_HeadPoseTracker*, bf_Confidence*>(
                tracker, confidence));
        m_ObservationMap.insert(
                pair<bicv_HeadPoseTracker*,bi_ObjectObservation*>(
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
