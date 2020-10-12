// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ARHeadPoseDynamicModel - dynamic model for head pose tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <vector>                                         // STL vector
#include <cmath>                                          // for exp
#include <boost/random/normal_distribution.hpp>           // Gaussian distr
#include <boost/random/variate_generator.hpp>             // sampling distr
#include <boost/math/distributions/normal.hpp>            // Gaussian distr

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPose.h>                      // head pose

// LOCAL INCLUDES
#include <bayes_image/bicv_ARHeadPoseDynamicModel.h>      // declaration of this
#include <bayes_image/bicv_Exceptions.h>                  // exceptions

using namespace std;
using namespace OpenCvPlus;

static const float MAX_POSE_EXCENTRICITY = 1.2;
static const float MIN_POSE_EXCENTRICITY = 0.8;
static const float MIN_POSE_SCALE = 0.1;
static const float MAX_POSE_SCALE = 1.2;

static const float hp2excentricity_table[] = {
        1.24615734672 , 1.19364693913 , 1.13563868902 , 1.05051473701 , 0.991837092486 , 0.951023350738 , 0.852209251569 ,
        1.28106763362 , 1.22136327119 , 1.16029961901 , 1.08921700104 , 1.02894960445 , 0.978823948707 , 0.868687658119 ,
        1.32746653747 , 1.2664085613 , 1.22781804168 , 1.16603844885 , 1.0967431767 , 1.02852098914 , 0.928065313228 ,
        1.35637073301 , 1.32613534883 , 1.28483866689 , 1.23293618626 , 1.17302469636 , 1.11357207435 , 1.02188963918 ,
        1.442077386 , 1.40440445232 , 1.35223148703 , 1.30955912257 , 1.25483304595 , 1.18209151382 , 1.04242022565 ,
        1.50260076438 , 1.45723543847 , 1.39972434384 , 1.34549143406 , 1.27602220745 , 1.20709046663 , 1.11764134687 ,
        1.48371228615 , 1.4423763976 , 1.38775090861 , 1.33118802526 , 1.27116179733 , 1.21928685838 , 1.09058072849 ,
        1.48122395478 , 1.44946921913 , 1.3681167258 , 1.35038841295 , 1.29652506797 , 1.21081252234 , 1.08715831127 ,
        1.45071899301 , 1.39586335221 , 1.34498752071 , 1.31408488298 , 1.25079630322 , 1.18044832418 , 1.06816658394 ,
        1.369950236 , 1.32658488391 , 1.26111714968 , 1.21228352559 , 1.15924127329 , 1.08060023755 , 0.986468104183 ,
        1.31110743788 , 1.26770288677 , 1.228084967 , 1.16297636599 , 1.10736145119 , 1.04573909494 , 0.946806532526 ,
        1.25177191788 , 1.22619501699 , 1.18832709675 , 1.12006485898 , 1.05858736656 , 1.00357216824 , 0.911989439674 ,
        1.23036580467 , 1.20160636636 , 1.1621581271 , 1.08613571276 , 1.03668425392 , 0.977318612808 , 0.925667841282
};
static const float hp2excentricity_stddev_table[] = {
        0.078890103793 , 0.0929963143346 , 0.0996614633257 , 0.0777682991653 , 0.0687166496346 , 0.073350311662 , 0.0752845863455 ,
        0.0766276836189 , 0.0982356163592 , 0.0951623834673 , 0.0840921577617 , 0.0844803184298 , 0.0871993527878 , 0.0721139885993 ,
        0.0679038573669 , 0.102017629375 , 0.0894484258707 , 0.0899038102437 , 0.103279333584 , 0.102624785131 , 0.0650685659498 ,
        0.0614118650128 , 0.0926946017473 , 0.087250159667 , 0.0897259593094 , 0.106704660195 , 0.102618048451 , 0.0858859726938 ,
        0.0565924823213 , 0.0891147756118 , 0.0862892120915 , 0.104364157635 , 0.103599091823 , 0.108035486066 , 0.0852982655896 ,
        0.0504245788574 , 0.0875972572474 , 0.09723890822 , 0.10559077393 , 0.111741296413 , 0.106898837668 , 0.0961135805718 ,
        0.0714776148084 , 0.0885739536414 , 0.0944729213081 , 0.101763550187 , 0.108639112103 , 0.0980279740557 , 0.0871138200461 ,
        0.0759627658355 , 0.0828484179852 , 0.0944589901779 , 0.09737546339 , 0.0991664313794 , 0.109181465188 , 0.0873217099327 ,
        0.0700931354225 , 0.106246438376 , 0.0990142608502 , 0.0969282789807 , 0.102488156027 , 0.10105827115 , 0.103326744014 ,
        0.0901675793522 , 0.0965718348722 , 0.0939255213318 , 0.0879312614518 , 0.106861539126 , 0.105494437144 , 0.0958853154037 ,
        0.0900989341659 , 0.0959119008907 , 0.0939239015089 , 0.088818634009 , 0.105746738865 , 0.111841542061 , 0.0905001197973 ,
        0.0854054830047 , 0.0968962197147 , 0.0893548022928 , 0.0987744285125 , 0.0954636226354 , 0.0888813993368 , 0.0870915154328 ,
        0.101495273778 , 0.0922808958094 , 0.0945686532002 , 0.0939512442072 , 0.0862434740476 , 0.0792318031493 , 0.0891588134891
};

namespace BICV {

typedef cvp_HeadPoseDiscreteDomain::real real;
typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

int bicv_ARHeadPoseDynamicModel::m_LabelCount = 0;

bicv_ARHeadPoseDynamicModel::bicv_ARHeadPoseDynamicModel(
    const bicv_ARHeadPoseDynamicModelParameters& params,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain) :
    m_Params(params), m_HeadPoseDomain(head_pose_domain),
    m_Label(++m_LabelCount) {

    initialize_head_pose_transition_matrix();

} // bicv_ARHeadPoseDynamicModel

/* static */ const bicv_ARHeadPoseDynamicModelParameters&
bicv_ARHeadPoseDynamicModel::parameters() const {
    return m_Params;
} // parameters

/* static */ void
bicv_ARHeadPoseDynamicModel::parameters(
        const bicv_ARHeadPoseDynamicModelParameters& params) {
    m_Params = params;
} // parameters

/* virtual */
bicv_ARHeadPoseDynamicModel::observation_type
bicv_ARHeadPoseDynamicModel::sample(
    bicv_ARHeadPoseDynamicModel::rng_engine_type& rng,
    const bicv_ARHeadPoseDynamicModel::state_type& state) const {

    boost::normal_distribution<value_type> gaussian_dist;
    boost::variate_generator<bicv_ARHeadPoseDynamicModel::rng_engine_type&,
        boost::normal_distribution<value_type> > gaussian(rng, gaussian_dist);

    // cout << "prev: " << state.m_HeadPoseParamsCur << endl;

    bicv_HeadPoseParameters params_rnd;
    params_rnd.m_Excentricity = gaussian();
    params_rnd.m_Scale = gaussian();
    params_rnd.m_TranslationX = gaussian();
    params_rnd.m_TranslationY = gaussian();
    params_rnd.m_HeadPose.pan(gaussian());
    params_rnd.m_HeadPose.tilt(gaussian());
    params_rnd.m_HeadPose.roll(gaussian());
//    cout << "sample: " << params_rnd << endl;

    boost::uniform_01<value_type> uniform_dist;
    boost::variate_generator<bicv_ARHeadPoseDynamicModel::rng_engine_type&,
        boost::uniform_01<value_type> > uniform(rng, uniform_dist);

    observation_type state_new;
    state_new.m_HeadPoseParamsPrev = state.m_HeadPoseParamsCur;
//    if (uniform() <= 0.1) {
        state_new.m_HeadPoseParamsCur =
            m_Params.m_MotConst +
            m_Params.m_ArPrev * state.m_HeadPoseParamsPrev +
            m_Params.m_ArCur * state.m_HeadPoseParamsCur +
            m_Params.m_StdDev * params_rnd;

        OpenCvPlus::cvp_HeadPose<> hp = m_HeadPoseDomain.discretize(
                state_new.m_HeadPoseParamsCur.m_HeadPose);
        int hp_id = m_HeadPoseDomain.id(hp);
        state_new.m_HeadPoseParamsCur.m_Excentricity =
                hp2excentricity_table[hp_id] +
                hp2excentricity_stddev_table[hp_id] *
                params_rnd.m_Excentricity;
//    } else {
//        state_new.m_HeadPoseParamsCur = m_Params.m_MotConst +
//            state.m_HeadPoseParamsCur + m_Params.m_StdDev * params_rnd;
//    }

    const vector<real>& pans = m_HeadPoseDomain.pan_values();
    const real maxpan = *max_element(pans.begin(), pans.end());
    const real minpan = *min_element(pans.begin(), pans.end());
    if (state_new.m_HeadPoseParamsCur.m_HeadPose.pan() > maxpan) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.pan(maxpan);
    } else if (state_new.m_HeadPoseParamsCur.m_HeadPose.pan() < minpan) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.pan(minpan);
    }

    const vector<real>& tilts = m_HeadPoseDomain.tilt_values();
    const real maxtilt = *max_element(tilts.begin(), tilts.end());
    const real mintilt = *min_element(tilts.begin(), tilts.end());
    if (state_new.m_HeadPoseParamsCur.m_HeadPose.tilt() > maxtilt) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.tilt(maxtilt);
    } else if (state_new.m_HeadPoseParamsCur.m_HeadPose.tilt() < mintilt) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.tilt(mintilt);
    }

    const vector<real>& rolls = m_HeadPoseDomain.roll_values();
    const real maxroll = *max_element(rolls.begin(), rolls.end());
    const real minroll = *min_element(rolls.begin(), rolls.end());
    if (state_new.m_HeadPoseParamsCur.m_HeadPose.roll() > maxroll) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.roll(maxroll);
    } else if (state_new.m_HeadPoseParamsCur.m_HeadPose.roll() < minroll) {
        state_new.m_HeadPoseParamsCur.m_HeadPose.roll(minroll);
    }

    if (state_new.m_HeadPoseParamsCur.m_Scale > MAX_POSE_SCALE) {
        state_new.m_HeadPoseParamsCur.m_Scale = MAX_POSE_SCALE;
    }

    if (state_new.m_HeadPoseParamsCur.m_Scale < MIN_POSE_SCALE) {
        state_new.m_HeadPoseParamsCur.m_Scale = MIN_POSE_SCALE;
    }

    if (state_new.m_HeadPoseParamsCur.m_Excentricity > MAX_POSE_EXCENTRICITY) {
        state_new.m_HeadPoseParamsCur.m_Excentricity = MAX_POSE_EXCENTRICITY;
    }

    if (state_new.m_HeadPoseParamsCur.m_Excentricity < MIN_POSE_EXCENTRICITY) {
        state_new.m_HeadPoseParamsCur.m_Excentricity = MIN_POSE_EXCENTRICITY;
    }
    state_new.m_Label = m_Label;

//    cout << "arprev " << m_Params.m_ArPrev << ", arcur " << m_Params.m_ArCur <<
//            ", stddev " << m_Params.m_StdDev << endl;

   // cout << "cur: " << state_new.m_HeadPoseParamsCur << endl;
    return state_new;

} // bicv_ARHeadPoseDynamicModel

/* virtual */
bicv_ARHeadPoseDynamicModel::value_type
bicv_ARHeadPoseDynamicModel::evaluate(
    const bicv_ARHeadPoseDynamicModel::state_type& state,
    const bicv_ARHeadPoseDynamicModel::observation_type& obs) const {

    boost::math::normal normal_distr;

    bicv_HeadPoseParameters params_unbiased =
        obs.m_HeadPoseParamsCur - m_Params.m_MotConst -
        m_Params.m_ArPrev * obs.m_HeadPoseParamsPrev -
        m_Params.m_ArCur * obs.m_HeadPoseParamsCur;

    params_unbiased /= m_Params.m_StdDev;
    float value =
            pdf(normal_distr, params_unbiased.m_Excentricity) *
            pdf(normal_distr, params_unbiased.m_Scale) *
            pdf(normal_distr, params_unbiased.m_TranslationX) *
            pdf(normal_distr, params_unbiased.m_TranslationY) *
            pdf(normal_distr, params_unbiased.m_HeadPose.pan()) *
            pdf(normal_distr, params_unbiased.m_HeadPose.tilt()) *
            pdf(normal_distr, params_unbiased.m_HeadPose.roll());
    return value;
}

void bicv_ARHeadPoseDynamicModel::initialize_head_pose_transition_matrix() {

    const vector<HeadPose>& head_poses = m_HeadPoseDomain.values();
    size_t head_pose_domain_size = head_poses.size();

    // initialize the transition table
    m_HeadPoseTransitionMatrix.resize(
        head_pose_domain_size * head_pose_domain_size);

    HeadPose src_pose; // transition source pose
    HeadPose tmp_pose; // temporary head pose used for evaluation

    // standard deviations for angle components
    const HeadPose std_dev_pose(m_Params.m_StdDev.m_HeadPose.pan(),
            m_Params.m_StdDev.m_HeadPose.tilt(),
            m_Params.m_StdDev.m_HeadPose.roll());

    value_type normalizationFactor;

    size_t transition_offset; // used for efficient index computation
    for (size_t src_pose_idx = 0; src_pose_idx < head_pose_domain_size;
            ++src_pose_idx) {

        src_pose = head_poses[src_pose_idx];
        transition_offset = src_pose_idx * head_pose_domain_size;
        normalizationFactor = 0;

        for (size_t dst_pose_idx = 0; dst_pose_idx < head_pose_domain_size;
            ++dst_pose_idx) {

            // normal distribution on the distance between head poses
            tmp_pose = (src_pose - head_poses[dst_pose_idx]) / std_dev_pose;
            tmp_pose *= tmp_pose / (-2);
            m_HeadPoseTransitionMatrix[transition_offset + dst_pose_idx] =
                exp(tmp_pose.pan() + tmp_pose.tilt() + tmp_pose.roll());
            // sum of transition probabilities
            normalizationFactor += m_HeadPoseTransitionMatrix[
                transition_offset + dst_pose_idx];
        }

        // normalize the transitions from src_pose
        m_HeadPoseTransitionMatrix[transition_offset] /=
                normalizationFactor;
        for (size_t dst_pose_idx = 1; dst_pose_idx < head_pose_domain_size;
            ++dst_pose_idx) {
            m_HeadPoseTransitionMatrix[transition_offset + dst_pose_idx] /=
                normalizationFactor;
            m_HeadPoseTransitionMatrix[transition_offset + dst_pose_idx] +=
                m_HeadPoseTransitionMatrix[transition_offset + dst_pose_idx
                                           - 1];
        }

    } // iterate over source poses

}

}
