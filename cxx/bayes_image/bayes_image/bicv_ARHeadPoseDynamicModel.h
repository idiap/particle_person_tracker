// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ARHeadPoseDynamicModel - dynamic model for head pose tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_ARHEADPOSEDYNAMICMODEL_H__
#define __BICV_ARHEADPOSEDYNAMICMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include <vector>                                         // STL vector

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>        // head pose domain

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>        // state

namespace BICV {

/// @brief Parameters of dynamic model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011
struct bicv_ARHeadPoseDynamicModelParameters {
    bicv_HeadPoseParameters m_StdDev; // independent std devs for all parameters
    bicv_HeadPoseParameters m_ArCur;  // AR for all parameters, for cur value
    bicv_HeadPoseParameters m_ArPrev; // AR for all parameters, for prev value
    bicv_HeadPoseParameters m_MotConst; // constant motion part
};

/// @brief Dynamic model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_ARHeadPoseDynamicModel : public BayesFilter::bf_ConditionalDistr
    <bicv_HeadPoseARTrackerState, bicv_HeadPoseARTrackerState,
    boost::mt19937> {

public:

    typedef BayesFilter::bf_ConditionalDistr<
        bicv_HeadPoseARTrackerState, bicv_HeadPoseARTrackerState,
        boost::mt19937> distribution_type;

    bicv_ARHeadPoseDynamicModel(const bicv_ARHeadPoseDynamicModelParameters&
        params, const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain);

    const bicv_ARHeadPoseDynamicModelParameters& parameters() const;

    void parameters(const bicv_ARHeadPoseDynamicModelParameters& params);

    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

private:

    void initialize_head_pose_transition_matrix();

    bicv_ARHeadPoseDynamicModelParameters m_Params;
    const OpenCvPlus::cvp_HeadPoseDiscreteDomain& m_HeadPoseDomain;
    std::vector<value_type> m_HeadPoseTransitionMatrix;
    const int m_Label;
    static int m_LabelCount;

};

}

#endif // __BICV_ARHEADPOSEDYNAMICMODEL_H__
