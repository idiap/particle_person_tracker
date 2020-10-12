// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseLikelihoodModel - likelihood model for head pose tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_HEADPOSELIKELIHOODMODEL_H__
#define __BICV_HEADPOSELIKELIHOODMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>              // head pose domain

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>        // state
#include <bayes_image/bicv_SkinHogObservation.h>          // compound obs
#include <bayes_image/bicv_HogModelTrainer.h>             // hog model trainer
#include <bayes_image/bicv_SkinModelTrainer.h>            // skin model trainer

namespace BICV {

/// @brief Parameters of likelihood model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011
struct bicv_HeadPoseLikelihoodModelParameters {
};

/// @brief Likelihood model for head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_HeadPoseLikelihoodModel : public BayesFilter::bf_ConditionalDistr
    <bicv_HeadPoseARTrackerState, bicv_SkinHogObservation,
    boost::mt19937> {

public:

    bicv_HeadPoseLikelihoodModel(
        const bicv_HeadPoseHogModel& trained_hog_model,
        const bicv_HeadPoseSkinModel& trained_skin_model,
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain);

    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

    //KLUDGE:
    value_type evaluate_HoG(const state_type& state,
            const observation_type& obs) const;
    value_type evaluate_Skin(const state_type& state,
            const observation_type& obs) const;
    //END_KLUDGE
private:

//    bicv_ARHeadPoseDynamicModelParameters m_Params;
    const OpenCvPlus::cvp_HeadPoseDiscreteDomain& m_HeadPoseDomain;
    const bicv_HeadPoseHogModel& m_TrainedHogModel;
    const bicv_HeadPoseSkinModel& m_TrainedSkinModel;

};

}

#endif // __BICV_HEADPOSELIKELIHOODMODEL_H__
