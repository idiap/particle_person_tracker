// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ARAffineMotionHeadPoseDynamicModel - dynamic model for head pose tracker
//     state that uses observations from the affine motion estimator
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_ARAFFINEMOTIONHEADPOSEDYNAMICMODEL_H__
#define __BICV_ARAFFINEMOTIONHEADPOSEDYNAMICMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include <vector>                                         // STL vector

// PROJECT INCLUDES
#include <bf_ConditionalDistr.h>                          // likelihood

// LOCAL INCLUDES
#include "bicv_HeadPoseTrackerState.h"                    // state
#include "bicv_HeadPoseDiscreteDomain.h"                  // head pose domain

namespace BICV {

/// @brief Parameters of dynamic model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011
struct bicv_ARAffineMotionHeadPoseDynamicModelParameters {
    bicv_HeadPoseParameters m_StdDev; // independent std devs for all parameters
    bicv_HeadPoseParameters m_ArCur;  // AR for all parameters, for cur value
    bicv_HeadPoseParameters m_ArPrev; // AR for all parameters, for prev value
};

/// @brief Dynamic model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_ARAffineMotionHeadPoseDynamicModel :
    public BayesFilter::bf_ConditionalDistr
        <bicv_HeadPoseARTrackerState, bicv_HeadPoseARTrackerState,
        boost::mt19937> {

public:

    bicv_ARAffineMotionHeadPoseDynamicModel(
            const bicv_ARHeadPoseDynamicModelParameters& params,
            const bicv_HeadPoseDiscreteDomain& head_pose_domain);

    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

private:

    void initialize_head_pose_transition_matrix();

    bicv_ARWithMotionHeadPoseDynamicModelParameters m_Params;
    const bicv_HeadPoseDiscreteDomain& m_HeadPoseDomain;
    std::vector<value_type> m_HeadPoseTransitionMatrix;

};

}

#endif // __BICV_ARWITHMOTIONHEADPOSEDYNAMICMODEL_H__
