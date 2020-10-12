// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_SkinHogObservationProvider - provides joint skin/HoG observations
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_SKINHOGOBSERVATIONPROVIDER_H__
#define __BICV_SKINHOGOBSERVATIONPROVIDER_H__

// PROJECT INCLUDES
#include <image_processing/ip_HogFeatureProducerRounded.h> // hog feature producer
#include <image_processing/ip_SkinFeatureProducer.h>       // skin feature producer

// LOCAL INCLUDES
#include <bayes_image/bicv_SkinHogObservation.h>           // joint observation
#include <bayes_image/bicv_HeadPoseTrackerState.h>         // tracker state

namespace BICV {

/// @brief Class that provides joint skin/HoG observations
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_SkinHogObservationProvider {

public:

    bicv_SkinHogObservationProvider(
            ImageProcessing::ip_HogFeatureProducerRounded * hog_feature_producer,
            ImageProcessing::ip_SkinFeatureProducer * skin_feature_producer);

    bicv_SkinHogObservation observation(
        const bicv_HeadPoseARTrackerState& state) const;

private:
    ImageProcessing::ip_HogFeatureProducerRounded * m_HogFeatureProducer;
    ImageProcessing::ip_SkinFeatureProducer * m_SkinFeatureProducer;
};

}

#endif // __BICV_SKINHOGOBSERVATIONPROVIDER_H__
