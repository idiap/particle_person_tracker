// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseLikelihoodModel - likelihood model for head pose tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseLikelihoodModel.h>           // likelihood model
#include <bayes_image/bicv_Exceptions.h>                        // exceptions
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // roi2state converter

using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace std;

// inverse weights used for HOG and skin parts of likelihood (must be positive)
// weight closer to zero means the component is more important
static const float HOG_WEIGHT = 10.0;
static const float SKIN_WEIGHT = 1.0;

//#define HOG_DISTANCE_L1
#define HOG_DISTANCE_L2
//#define HOG_DISTANCE_MAHALANOBIS
//#define HOG_DISTANCE_CHI2

#define SKIN_DISTANCE_EMPIRICAL
//#define SKIN_DISTANCE_L1
//#define SKIN_DISTANCE_L2
//#define SKIN_DISTANCE_MAHALANOBIS

namespace BICV {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

bicv_HeadPoseLikelihoodModel::bicv_HeadPoseLikelihoodModel(
        const bicv_HeadPoseHogModel& trained_hog_model,
        const bicv_HeadPoseSkinModel& trained_skin_model,
        const cvp_HeadPoseDiscreteDomain& head_pose_domain) :
        m_HeadPoseDomain(head_pose_domain),
        m_TrainedHogModel(trained_hog_model),
        m_TrainedSkinModel(trained_skin_model) {
}

/* virtual */ bicv_HeadPoseLikelihoodModel::observation_type
bicv_HeadPoseLikelihoodModel::sample(rng_engine_type& rng,
        const state_type& state) const {
    throw bicv_Exception("Not implemented!");
}

/* virtual */ bicv_HeadPoseLikelihoodModel::value_type
bicv_HeadPoseLikelihoodModel::evaluate(const state_type& state,
        const observation_type& obs) const {

    value_type p_hog = evaluate_HoG(state, obs);
    value_type p_skin = evaluate_Skin(state, obs);
    return p_hog * p_skin;
}

static const double HISTOGRAM_TEMPLATE_DISTANCE_THRESHOLD = 1.0;

bicv_HeadPoseLikelihoodModel::value_type
bicv_HeadPoseLikelihoodModel::evaluate_HoG(const state_type& state,
        const observation_type& obs) const {

    HeadPose head_pose = state.m_HeadPoseParamsCur.m_HeadPose;
    head_pose = m_HeadPoseDomain.discretize(head_pose);
    int head_pose_ID = m_HeadPoseDomain.id(head_pose);

    const std::vector<ImageProcessing::ip_HistogramTemplate>&
        hog_mean_templates = m_TrainedHogModel.mean_templates();

    #ifdef HOG_DISTANCE_L1
    value_type d = distance_L1(obs.m_HogFeature,
            hog_mean_templates[head_pose_ID], 1.0f);
    #endif
    #ifdef HOG_DISTANCE_L2
    value_type d = distance_L2_squared(obs.m_HogFeature,
            hog_mean_templates[head_pose_ID], 1.0f);
    #endif
    #ifdef HOG_DISTANCE_MAHALANOBIS
    value_type d = distance_Mahalanobis_squared(obs.m_HogFeature,
            hog_mean_templates[head_pose_ID], );
    #endif
    #ifdef HOG_DISTANCE_CHI2
    value_type d = distance_Chi2(obs.m_HogFeature,
            hog_mean_templates[head_pose_ID]);
    #endif

    value_type pd = exp(-d * HOG_WEIGHT);
    return pd;
}

bicv_HeadPoseLikelihoodModel::value_type
bicv_HeadPoseLikelihoodModel::evaluate_Skin(const state_type& state,
        const observation_type& obs) const {

    HeadPose head_pose = state.m_HeadPoseParamsCur.m_HeadPose;
    head_pose = m_HeadPoseDomain.discretize(head_pose);
    int head_pose_ID = m_HeadPoseDomain.id(head_pose);

    const std::vector<ImageProcessing::ip_SkinTemplate>&
        skin_mean_templates = m_TrainedSkinModel.mean_templates();

    #ifdef SKIN_DISTANCE_EMPIRICAL
    value_type pd = distance_emp(obs.m_SkinFeature,
            skin_mean_templates[head_pose_ID]);
    #endif
    #ifdef SKIN_DISTANCE_L1
    value_type d = distance_L1(obs.m_SkinFeature,
            skin_mean_templates[head_pose_ID]);
    value_type pd = exp(-d * SKIN_WEIGHT);
    #endif
    #ifdef SKIN_DISTANCE_L2
    value_type d = distance_L2_squared(obs.m_SkinFeature,
            skin_mean_templates[head_pose_ID]);
    value_type pd = exp(-d * SKIN_WEIGHT);
//    #elif SKIN_DISTANCE_MAHALANOBIS
//    value_type d = distance_mahalanobis(obs.m_SkinFeature,
//            skin_mean_templates[head_pose_ID]);
//    value_type pd = exp(-d * SKIN_WEIGHT);
    #endif
    return pd;
}


//    bicv_ARHeadPoseDynamicModelParameters m_Params;
//    const bicv_HeadPoseDiscreteDomain& m_HeadPoseDomain;
//    const bicv_HogModelTrainer& m_TrainedHogModel;
//    const bicv_SkinModelTrainer& m_TrainedSkinModel;

}
