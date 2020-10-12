// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_GeneralDynamicModel - dynamic model for general tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <vector>                                         // STL vector
#include <utility>                                        // STL make_pair
#include <cmath>                                          // for exp
#include <boost/random/normal_distribution.hpp>           // Gaussian distr
#include <boost/random/variate_generator.hpp>             // sampling distr

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralDynamicModel.h>       // declaration of this
#include <bayes_image/bicv_Exceptions.h>                // exceptions

using namespace std;
using namespace BayesFilter;

static const float MAX_POSE_EXCENTRICITY = 1.5;
static const float MIN_POSE_EXCENTRICITY = 0.8;
static const float MIN_POSE_SCALE = 0.1;
static const float MAX_POSE_SCALE = 1.2;

namespace BICV {

int bicv_GeneralDynamicModel::m_LabelCount = 0;

bicv_GeneralDynamicModel::bicv_GeneralDynamicModel(
    const bicv_GeneralDynamicModelParameters& params,
    unsigned num_poses) :
    m_Params(params), m_NumPoses(num_poses), m_Label(++m_LabelCount) {

    m_PoseDistributionElements.reserve(m_NumPoses);
    for (unsigned idx = 0; idx < m_NumPoses; ++idx) {
        m_PoseDistributionElements.push_back(make_pair(idx, 1.0 / m_NumPoses));
    }
} // bicv_GeneralDynamicModel

const bicv_GeneralDynamicModelParameters&
bicv_GeneralDynamicModel::parameters() {
    return m_Params;
} // parameters

void
bicv_GeneralDynamicModel::parameters(
        const bicv_GeneralDynamicModelParameters& params) {
    m_Params = params;
} // parameters

/* virtual */
bicv_GeneralDynamicModel::observation_type
bicv_GeneralDynamicModel::sample(
    bicv_GeneralDynamicModel::rng_engine_type& rng,
    const bicv_GeneralDynamicModel::state_type& state) const {

    boost::normal_distribution<value_type> gaussian_dist;
    boost::variate_generator<bicv_GeneralDynamicModel::rng_engine_type&,
        boost::normal_distribution<value_type> > gaussian(rng, gaussian_dist);

//    cout << "prev: " << state.m_HeadPoseParamsCur << endl;

    // sample gaussian random parameters
    bicv_TrackerParameters params_rnd;
    params_rnd.m_Excentricity = gaussian();
    params_rnd.m_Scale = gaussian();
    params_rnd.m_TranslationX = gaussian();
    params_rnd.m_TranslationY = gaussian();
//    cout << "sample: " << params_rnd << endl;

    // apply dynamics
    observation_type state_new;
    state_new.m_ParamsPrev = state.m_ParamsCur;
    state_new.m_ParamsCur =
        m_Params.m_MotConst +
        m_Params.m_ArPrev * state.m_ParamsPrev +
        m_Params.m_ArCur  * state.m_ParamsCur +
        m_Params.m_StdDev * params_rnd;

    // sample discrete pose
    if (m_NumPoses > 1) {
        std::vector<std::pair<unsigned, float> > pose_distr_elts =
                m_PoseDistributionElements;
        static const float proba_self = 10.0;
        static const float proba_others = 1.0;
        for (unsigned idx = 0; idx < m_NumPoses; ++idx) {
            pose_distr_elts[idx].second = proba_others;
        }
        pose_distr_elts[state.m_ParamsCur.m_PoseIndex].second = proba_self;
        bf_DiscreteDistribution<unsigned> pose_distribution(pose_distr_elts);
        state_new.m_ParamsCur.m_PoseIndex = BayesFilter::sample(
                rng, pose_distribution);
    } else {
        state_new.m_ParamsCur.m_PoseIndex = 0;
    }

    // correct sample
    if (state_new.m_ParamsCur.m_Scale > MAX_POSE_SCALE) {
        state_new.m_ParamsCur.m_Scale = MAX_POSE_SCALE;
    }

    if (state_new.m_ParamsCur.m_Scale < MIN_POSE_SCALE) {
        state_new.m_ParamsCur.m_Scale = MIN_POSE_SCALE;
    }

    if (state_new.m_ParamsCur.m_Excentricity > MAX_POSE_EXCENTRICITY) {
        state_new.m_ParamsCur.m_Excentricity = MAX_POSE_EXCENTRICITY;
    }

    if (state_new.m_ParamsCur.m_Excentricity < MIN_POSE_EXCENTRICITY) {
        state_new.m_ParamsCur.m_Excentricity = MIN_POSE_EXCENTRICITY;
    }
    state_new.m_Label = m_Label;

//    cout << "arprev " << m_Params.m_ArPrev << ", arcur " << m_Params.m_ArCur <<
//            ", stddev " << m_Params.m_StdDev << endl;

//    cout << "cur: " << state_new.m_HeadPoseParamsCur << endl;
    return state_new;

} // bicv_ARHeadPoseDynamicModel

/* virtual */
bicv_GeneralDynamicModel::value_type
bicv_GeneralDynamicModel::evaluate(
    const bicv_GeneralDynamicModel::state_type& state,
    const bicv_GeneralDynamicModel::observation_type& obs) const {

    throw bicv_Exception("Not implemented!");
}

} // namespace BICV
