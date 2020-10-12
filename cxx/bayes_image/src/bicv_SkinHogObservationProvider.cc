// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_SkinHogObservationProvider - provides joint skin/HoG observations
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <bayes_image/bicv_SkinHogObservationProvider.h>        // declaration of this
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // state2roi converter

using namespace ImageProcessing;

namespace BICV {

bicv_SkinHogObservationProvider::bicv_SkinHogObservationProvider(
        ImageProcessing::ip_HogFeatureProducerRounded * hog_feature_producer,
        ImageProcessing::ip_SkinFeatureProducer * skin_feature_producer) :
        m_HogFeatureProducer(hog_feature_producer),
        m_SkinFeatureProducer(skin_feature_producer) {
} // bicv_SkinHogObservationProvider

bicv_SkinHogObservation bicv_SkinHogObservationProvider::observation(
        const bicv_HeadPoseARTrackerState& state) const {
    ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
            state.m_HeadPoseParamsCur);
    return bicv_SkinHogObservation (
            m_HogFeatureProducer->compute_feature(roi),
            m_SkinFeatureProducer->compute_feature(roi)
    );
} // observation

}
