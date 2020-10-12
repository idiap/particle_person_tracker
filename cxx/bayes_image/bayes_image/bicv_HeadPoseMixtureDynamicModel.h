/**
 * @file cxx/bayes_image/bayes_image/bicv_HeadPoseMixtureDynamicModel.h
 * @date 03 December 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief A mixture of dynamic models used in head pose tracker
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_HEADPOSEMIXTUREDYNAMICMODEL_H__
#define __BICV_HEADPOSEMIXTUREDYNAMICMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include "boost/date_time/posix_time/posix_time.hpp"      // boost posix time
#include <vector>                                         // STL vector
#include <list>                                           // STL list

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <bayes_filter/bf_ConditionalDistrMixture.h>      // mixture
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>        // head pose domain
#include <opencvplus/cvp_FaceDetectorStatisticsStorage.h>
#include <opencvplus/cvp_FaceDescriptor.h>                // face descriptor
#include <image_processing/ip_MotionParameters.h>         // motion params

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>        // state
#include <bayes_image/bicv_HeadPoseMixtureDynamicModelConfig.h> // config
#include <bayes_image/bicv_ARHeadPoseDynamicModel.h>      // dynamic model

namespace BICV {

/// @brief Mixture of dynamic models for head pose tracker state.
/// This mixture consists of four parts: random search dynamics,
/// auto-regressive dynamics, face detection-based dynamics and
/// motion estimation-based dynamics.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.12.2012

class bicv_HeadPoseMixtureDynamicModel : public BayesFilter::bf_ConditionalDistr
    <bicv_HeadPoseARTrackerState, bicv_HeadPoseARTrackerState,
    boost::mt19937> {

public:

    typedef BayesFilter::bf_ConditionalDistr<
        bicv_HeadPoseARTrackerState, bicv_HeadPoseARTrackerState,
        boost::mt19937> distribution_type;

    typedef BayesFilter::bf_ConditionalDistrMixture<
        distribution_type::state_type,
        distribution_type::observation_type,
        distribution_type::rng_engine_type,
        distribution_type::value_type,
        distribution_type::policy_type
    > distribution_mixture_type;

    // LIFECYCLE

    /// Constructor, initializes this dynamic model mixture by instantiating
    /// the components - four dynamic models
    /// @param config Dynamic model configuration
    /// @param head_pose_domain Head pose domain instance
    bicv_HeadPoseMixtureDynamicModel(
            const BICV::bicv_HeadPoseMixtureDynamicModelConfig& config,
            const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain,
            const OpenCvPlus::cvp_FaceDetectorStatisticsStorage&
                face_detector_stats);

    /// Destructor, deinitializes this dynamic model destroying dynamic model
    /// components
    virtual ~bicv_HeadPoseMixtureDynamicModel();

    /// Prepares dynamic model parameters for sampling or evaluation of
    /// the mixture of dynamic model distributions
    /// @param sampling_time Current time for which to sample the state
    /// @param previous_sampling_time Current time for which to sample the state
    /// @param motion_parameters Estimated motion parameters to use in sampling
    /// @param previous_sampling_time Flags indicating whether motion parameters
    /// are valid
    /// @param associated_faces Faces associated with the current state
    void prepare(const boost::posix_time::ptime& sampling_time,
        const boost::posix_time::ptime& previous_sampling_time,
        const ImageProcessing::ip_MotionParameters& motion_parameters,
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& associated_faces);

    /// Overridden base class method. Produces a new sample based on the
    /// provided state and using the provided random number generator.
    /// Uses internally prepared sampling parameters such as current timestamp.
    /// @param rng Random number generator to use for sampling
    /// @param state State to be used for conditional sampling
    /// @see prepare_sampling
    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    /// Overridden base class method. Calculates the probability of the
    /// current observation given the provided state.
    /// Uses internally prepared sampling parameters such as current timestamp.
    /// @param state State to be used for probability evaluation
    /// @param obs Observation for which to evaluate the probability
    /// @see prepare_sampling
    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

    /// Calculates the probability of current state given previous state.
    /// Uses internally prepared sampling parameters such as current timestamp.
    /// @param state_old Previous state
    /// @param state_new Current state
    value_type evaluate_dynamics(const state_type& state_old,
            const state_type& state_new) const;

    /// Calculates the probability of current state as given by proposal
    /// distribution.
    /// Uses internally prepared sampling parameters such as current timestamp.
    /// @param state_old Previous state
    /// @param state_new Current state
    value_type evaluate_proposal(const state_type& state_old,
            const state_type& state_new) const;
private:

    // create random search dynamic model
    BICV::bicv_ARHeadPoseDynamicModel * create_RS_dynamic_model();
    // create auto-regressive dynamic model
    BICV::bicv_ARHeadPoseDynamicModel * create_AR_dynamic_model();
    // create motion based dynamic model
    BICV::bicv_ARHeadPoseDynamicModel * create_MB_dynamic_model();
    // create face based dynamic model
    BICV::bicv_ARHeadPoseDynamicModel * create_FB_dynamic_model();

    // derive standard deviation parameters based on current and previous
    // timestamps (each parameter field contains standard deviation of the
    // corresponding component)
    bicv_HeadPoseParameters derive_stddev_parameters(
            const boost::posix_time::ptime& sampling_time,
            const boost::posix_time::ptime& previous_sampling_time);

    // derive head pose dynamic model parameters (mostly constant and
    // standard deviation parts) using face detector statistics
    bicv_ARHeadPoseDynamicModelParameters
    derive_parameters_from_face_detections(
            BICV::bicv_ARHeadPoseDynamicModel * fb_dynamic_model,
            const std::list<OpenCvPlus::cvp_FaceDescriptor>& face_detections);

    // derive head pose dynamic model parameters (mostly constant and
    // standard deviation parts) using face detector statistics
    bicv_ARHeadPoseDynamicModelParameters
    derive_parameters_from_motion(
            BICV::bicv_ARHeadPoseDynamicModel * mb_dynamic_model,
            const ImageProcessing::ip_MotionParameters& motion_parameters);

    // reference to head pose domain
    const OpenCvPlus::cvp_HeadPoseDiscreteDomain& m_HeadPoseDomain;
    // reference to face detector statistics storage
    const OpenCvPlus::cvp_FaceDetectorStatisticsStorage&
        m_FaceDetectorStatisticsStorage;
    // configuration options for this dynamic model mixture
    const BICV::bicv_HeadPoseMixtureDynamicModelConfig& m_Config;
    // aggregated mixture of distributions
    distribution_mixture_type * m_DynamicModel;

    // minimum standard deviation of head pose tracker parameters
    bicv_HeadPoseParameters m_MinStddevParams;
    // maximum standard deviation of head pose tracker parameters
    bicv_HeadPoseParameters m_MaxStddevParams;

    boost::posix_time::ptime m_SamplingTime;
};

}

#endif // __BICV_HEADPOSEMIXTUREDYNAMICMODEL_H__
