// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ColourSegmentationTracker - class to track colour-based segmentation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_COLOURSEGMENTATIONTRACKER_H__
#define __BICV_COLOURSEGMENTATIONTRACKER_H__

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralTrackerState.h>        // state
#include <bayes_image/bicv_StateImageLikelihood.h>       // likelihood
#include <bayes_image/bicv_CsMixtureDynamicModel.h>      //

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>
#include <boost/circular_buffer.hpp>                // boost circular buffer
#include <boost/math/distributions/normal.hpp>      // normal distribution
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <cmotion2d/CMotion2DModel.h>
#include <cmotion2d/CMotion2DEstimator.h>
#include <image_processing/ip_ImageProvider.h>             // image provider
#include <image_processing/ip_Dense2DMotionProcessorInria.h> // motion processor
#include <bayes_filter/bf_DiscreteDistribution.h>          // particle set
#include <bayes_filter/bf_ConditionalDistr.h>              // likelihood
#include <utils/ut_DummyLogger.h>                          // default logger
#include <opencvplus/FaceColorModel.h>                     // face color model
#include <opencvplus/cvp_FaceDescriptor.h>                  // FD descriptor

namespace BICV {

/// @brief Class to track objects using a trained colour model
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_ColourSegmentationTracker {

public:

    typedef BayesFilter::bf_ConditionalDistr<bicv_TrackerState,
        bicv_TrackerState, boost::mt19937> DynamicModel;
    typedef bicv_StateImageLikelihood Likelihood;
    typedef BayesFilter::bf_DiscreteDistribution<bicv_TrackerState>
        ParticleDistribution;

    bicv_ColourSegmentationTracker(
        bicv_CsMixtureDynamicModel * dyn_model,
        ImageProcessing::ip_ImageProvider * data_provider,
        ImageProcessing::ip_Dense2DMotionProcessorInria * motion_processor,
        FaceColorModel::FaceColorModel * face_colour_model,
        unsigned num_particles);

    ~bicv_ColourSegmentationTracker();

    // OPERATIONS

    ImageProcessing::ip_ImageProvider * data_provider() const {
        return m_DataProvider;
    }

    /// Get tracker's ID
    int id() const {
        return m_Id;
    }
    /// Set tracker's ID
    void id(int value) {
        m_Id = value;
    }

    /// Initialize particle distribution to a distribution associated
    /// with the detected face.
    /// @param init_candidate Face detection to be used for initialization
    /// @param timestamp Timestamp of the initialization
    void init(const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
            const boost::posix_time::ptime & timestamp);

    /// Associate a detected face with the current state of a tracker. Tracker
    /// uses the provided information to establish prior distribution over
    /// current bounding box and head pose.
    /// @param face_descriptor Descriptor of the observed face
    void observe_face(const OpenCvPlus::cvp_FaceDescriptor& face_descriptor);

    /// Provide read access to a number of last associated face detections.
    /// @return A collection of face detections that have been associated
    /// to the tracker
    const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor> &
        associated_face_detections() const;

    /// Provides a list of associated face detections for a given timestamp.
    /// @param timestamp Timestamp for which to provide face detections
    /// @param duration Time interval to consider; defaults to 0
    /// @return A collection of associated face detections that fall into
    /// the provided time interval
    std::list<OpenCvPlus::cvp_FaceDescriptor>
        associated_face_detections_for_timestamp(
                const boost::posix_time::ptime& timestamp,
                const boost::posix_time::time_duration& duration =
                        boost::posix_time::seconds(0)) const;

    /// Returns face colour model associated with the tracker
    /// @return Face colour model associated with the tracker
    FaceColorModel::FaceColorModel * colour_model() const {
        return m_FaceColourModel;
    }

    /// Return the likelihood model which can be used to evaluate likelihoods
    /// on tracker states
    /// @return Likelihood model for a colour segmentation based tracker
    const Likelihood * likelihood() const {
        return m_Likelihood;
    }

    void iterate();

    bool should_continue_tracking() const;

    const ParticleDistribution& particle_distribution() const {
        return m_ParticleDistribution;
    }

    float mode_likelihood() const;

    /// Prior distribution on tracker scale
    /// @return prior distribution on tracker scale
    const boost::math::normal_distribution<float>& scale_prior() const {
        return m_ScaleAverageBasedPrior;
    }

private:

    void invalidate_providers();
    inline unsigned generate_random_seed() const;
    inline float weight_correction(const bicv_TrackerState& state) const;
    inline float weight_correction_prior(const bicv_TrackerState& state) const;
    inline float weight_correction_face(const bicv_TrackerState& state) const;
    void update_skin_colour();
    float internal_tracking_score() const;

    /// derives tracker scale distribution based on a collection of face
    /// detections
    /// @param
    void derive_average_scale_distribution(
        const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor>& faces,
        float min_scale_stddev);

    bicv_TrackerState correct_state_by_skin_region(IplImage * ipImage,
            const bicv_TrackerState& state);

    // adjust state so that the corresponding ROI lies in the original image
    inline bicv_TrackerState correct_state(const bicv_TrackerState& state);

    /// Derive (or sample) tracker pose index from face detection orientation
    /// (typecally frontal, left profile, right profile)
    /// @param face_pose Face orientation (frontal, left/right profile)
    /// @return Tracker pose index (@see bicv_TrackerParameters)
    unsigned pose_index_from_face_orientation(
            OpenCvPlus::cvp_FaceDetectorPose face_pose) const;

    ParticleDistribution m_ParticleDistribution;  // particle distribution
    boost::mt19937 m_ParticleDistributionRng;     // random number generator

    bicv_CsMixtureDynamicModel * m_DynamicModel;  // dynamic model
    boost::mt19937 m_DynamicModelRng;             // random number generator

    const Likelihood * m_Likelihood;              // likelihood model

    // motion processor components
    CMotion2DModel * m_MotionModel;
    CMotion2DEstimator * m_MotionEstimator;
    ImageProcessing::ip_Dense2DMotionProcessorInria * m_MotionProcessor;


    // prior distribution on head scale
    boost::math::normal_distribution<float> m_ScaleAverageBasedPrior;
    // keeps a number of last associated face detections to establish
    // prior distributions over the state
    boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor>
        m_LastAssociatedFaceDetections;

    int m_Id;

    // source image provider
    ImageProcessing::ip_ImageProvider * m_DataProvider;
    // face colour model
    FaceColorModel::FaceColorModel * m_FaceColourModel;

    // circular buffer of likelihood values to develop stopping criterion
    boost::circular_buffer<float> m_LhoodValues;

    // KLUDGE: logging for trackers
    std::ofstream m_Dumpfile;
    int m_IterationCount;
    // END_KLUDGE


};

} // namespace BICV

#endif // __BICV_COLOURSEGMENTATIONTRACKER_H__
