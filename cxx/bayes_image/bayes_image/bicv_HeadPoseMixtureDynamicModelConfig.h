/**
 * @file cxx/bayes_image/bayes_image/bicv_HeadPoseMixtureDynamicModelConfig.h
 * @date 03 December 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Configuration options for a mixture of dynamic models
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_HEADPOSEMIXTUREDYNAMICMODELCONFIG_H__
#define __BICV_HEADPOSEMIXTUREDYNAMICMODELCONFIG_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include "boost/date_time/posix_time/posix_time.hpp"      // boost posix time
#include <vector>                                         // STL vector

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>        // head pose domain

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>        // state

namespace BICV {

/// @brief Configuration options for a mixture of dynamic models.
/// Includes relative weights for mixture components.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.12.2012

struct bicv_HeadPoseMixtureDynamicModelConfig {

    /// Random search dynamic model component weight
    float m_RSWeight;
    /// Auto-regressive dynamic model component weight
    float m_ARWeight;
    /// Motion-based dynamic model component weight
    float m_MBWeight;
    /// Face-based dynamic model component weight
    float m_FBWeight;

};

}

#endif // __BICV_HEADPOSEMIXTUREDYNAMICMODELCONFIG_H__
