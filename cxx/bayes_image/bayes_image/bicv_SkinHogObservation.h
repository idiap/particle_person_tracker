// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_SkinHogObservation - class for a joint skin/HoG observation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_SKINHOGOBSERVATION_H__
#define __BICV_SKINHOGOBSERVATION_H__

// PROJECT INCLUDES
#include <image_processing/ip_HistogramTemplate.h>    // hog feature
#include <image_processing/ip_SkinFeatureProducer.h>  // skin feature producer

namespace BICV {

/// @brief Class for a joint skin/HoG observation
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct bicv_SkinHogObservation {
    bicv_SkinHogObservation(
            const ImageProcessing::ip_HistogramTemplate& hog_feature,
            const ImageProcessing::ip_SkinTemplate& skin_feature) :
            m_HogFeature(hog_feature), m_SkinFeature(skin_feature) {
    }
    const ImageProcessing::ip_HistogramTemplate& m_HogFeature;
    const ImageProcessing::ip_SkinTemplate& m_SkinFeature;
};

}

#endif // __BICV_SKINHOGOBSERVATION_H__
