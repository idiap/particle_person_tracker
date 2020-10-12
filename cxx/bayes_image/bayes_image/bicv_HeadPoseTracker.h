// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseTracker - class to track head pose
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_HEADPOSETRACKER_H__
#define __BICV_HEADPOSETRACKER_H__

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>       // state
#include <bayes_image/bicv_SkinHogObservation.h>         // observation
#include <bayes_image/bicv_SkinHogObservationProvider.h> // obs provider
#include <bayes_image/bicv_SkinModelTrainer.h>           // skin model trainer
#include <bayes_image/bicv_HeadPoseMixtureDynamicModel.h>


// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>
#include <boost/circular_buffer.hpp>                // boost circular buffer
#include <boost/math/distributions/normal.hpp>      // normal distribution
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <cmotion2d/CMotion2DModel.h>

#include <bayes_filter/bf_DiscreteDistribution.h>            // particle set
#include <bayes_filter/bf_ConditionalDistr.h>                // likelihood
#include <image_processing/ip_GradientImageProcessor.h>      // gradient provider
#include <image_processing/ip_IntegralGradientHistogramProcessor.h> // integral hog provider
#include <image_processing/ip_SkinColourProcessor.h>         // skin colour provider
#include <image_processing/ip_IntegralSkinMaskProcessor.h>   // integral skin provider
#include <image_processing/ip_HogFeatureProducerRounded.h>   // HoG feature producer
#include <image_processing/ip_SkinFeatureProducer.h>         // skin feature producer
#include <image_processing/ip_ColourSkinFeatureProducer.h>   // for skin model update
#include <image_processing/ip_Dense2DMotionProcessorInria.h> // motion processor
#include <image_processing/ip_MotionParameters.h>            // motion params

#include <utils/ut_DummyLogger.h>                           // default logger
#include <opencvplus/FaceColorModel.h>                      // face color model
#include <opencvplus/cvp_FaceDetectorStatisticsStorage.h>   // FD stats
#include <opencvplus/cvp_FaceDescriptor.h>                  // FD descriptor

namespace BICV {

/// @brief Class to track head pose
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_HeadPoseTracker {

public:

    typedef BayesFilter::bf_ConditionalDistr<bicv_HeadPoseARTrackerState,
        bicv_HeadPoseARTrackerState, boost::mt19937> DynamicModel;
    typedef BayesFilter::bf_ConditionalDistr<bicv_HeadPoseARTrackerState,
        bicv_SkinHogObservation, boost::mt19937> Likelihood;
    typedef BayesFilter::bf_DiscreteDistribution<bicv_HeadPoseARTrackerState>
        ParticleDistribution;

    // LIFECYCLE

    /// Constructor
    bicv_HeadPoseTracker(bicv_HeadPoseMixtureDynamicModel * dyn_model,
        const Likelihood * lhood,
        ImageProcessing::ip_ImageProvider * data_provider,
        ImageProcessing::ip_ImageProvider * grayscale_provider,
        ImageProcessing::ip_GradientImageProcessor * gradient_provider,
        ImageProcessing::ip_ColourSkinFeatureProducer * skin_colour_producer,
        ImageProcessing::ip_Dense2DMotionProcessorInria * motion_processor,
        FaceColorModel::FaceColorModel * face_colour_model,
        ImageProcessing::ip_ImageProvider * depth_provider,
        bicv_HeadPoseSkinModel * trained_skin_model,
        const ImageProcessing::ip_HogFeatureParameters& hog_params,
        const ImageProcessing::ip_SkinFeatureParameters& skin_params,
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain,
        const OpenCvPlus::cvp_FaceDetectorStatisticsStorage * fd_stats_storage,
        unsigned num_particles);

    /// Destructor
    ~bicv_HeadPoseTracker();

    // OPERATIONS
    ImageProcessing::ip_ImageProvider * data_provider() const {
        return m_DataProvider;
    }

    /// Get tracker's ID
    int id() const {
        return m_Id;
    }
    /// Set tracker's ID
//    void id(int value) {
//        m_Id = value;
//    }

    /// Initialize particle distribution to a degenerate distribution
    /// containing particles equal to the given one, all with the same
    /// probability.
    /// @param state State to be used to initialize all the particles
    void init(const bicv_HeadPoseARTrackerState& state);

    /// Initialize particle distribution to a distribution associated
    /// with the detected face.
    /// @param init_candidate Face detection to be used for initialization
    /// @param timestamp Timestamp of the initialization
    void init(const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
        const boost::posix_time::ptime & timestamp);

    void init_with_unknown_head_pose(const bicv_HeadPoseARTrackerState& state);

    FaceColorModel::FaceColorModel * face_color_model() {
        return mFaceColourModel;
    }

    inline void set_face_idx(int i)
    {
      m_FaceIdx = i;
    }

    inline int get_face_idx() const { return m_FaceIdx; }

    void set_skin_colour_model(const ImageProcessing::ip_RoiWindow&
            roi);
    void set_skin_colour_model(const OpenCvPlus::cvp_SkinColourModel&
            skin_colour_model);
    void update_skin_colour_model(const OpenCvPlus::cvp_SkinColourModel&
            skin_colour_model);

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

    /// Main tracker update method. Performs one iteration of a particle filter:
    /// samples dynamic model and weights the particles.
    void iterate();

    /// Evaluate tracker scene coordinates. Method uses region of interest
    /// that corresponds to the mean of tracker particles to estimate
    /// 3D scene coordinates. The estimation is based either on external depth
    /// provider (if it is available) or on dummy calibration parameters and
    /// tracker scale (using scale to depth estimation table)
    CvScalar scene_coordinates_3D() const;

    bool should_continue_tracking() const;

    ImageProcessing::ip_SkinColourProcessor * skin_colour_processor() {
        return m_SkinColourProvider;
    }

    const ParticleDistribution& particle_distribution() const {
        return m_ParticleDistribution;
    }

    /// Return the likelihood model which can be used to evaluate likelihoods
    /// on tracker states, given HOG and skin features
    /// @return Likelihood model for a skin/hog based head pose tracker
    const Likelihood * likelihood() const {
        return m_Likelihood;
    }

    /// Return the HOG feature producer to be able to compute HOG features
    /// for image ROI
    /// @return HOG feature producer
    ImageProcessing::ip_HogFeatureProducerRounded *
    hog_feature_producer() {
        return m_HogFeatureProducer;
    }

    /// Return the skin feature producer to be able to compute skin features
    /// for image ROI
    /// @return Skin feature producer
    ImageProcessing::ip_SkinFeatureProducer *
    skin_feature_producer() {
        return m_SkinFeatureProducer;
    }

    float mode_likelihood() const;

    /// Prior distribution on tracker scale
    /// @return prior distribution on tracker scale
    const boost::math::normal_distribution<float>& scale_prior() const {
        return m_ScaleAverageBasedPrior;
    }

    /// Timestamped motion estimation parameters, timestamps are given in a
    /// number of microseconds
    typedef std::pair<double, ImageProcessing::ip_MotionParameters>
        TimedMotionEstim;

    /// Provides the last motion estimates.
    /// @return List of motion estimation values
    const std::list<TimedMotionEstim>& motion_estimation_history() const {
        return m_MotionEstimList;
    }

private:

    void invalidate_providers();
    inline unsigned generate_random_seed() const;
    inline float weight_correction(const bicv_HeadPoseARTrackerState& state)
        const;
    inline float weight_correction_prior(const bicv_HeadPoseARTrackerState& state)
        const;
    inline float weight_correction_face(const bicv_HeadPoseARTrackerState& state)
        const;

    /// derives tracker scale distribution based on a collection of
    /// face detections
    /// @param
    void derive_average_scale_distribution(
        const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor>& faces,
        float min_scale_stddev);

    inline void correct_particle(
            bicv_HeadPoseARTrackerState& particle, int width, int height);

    void update_skin_colour();
    float internal_tracking_score() const;

    // Revises motion estimation list removing outdated measurements.
    // A measurement is considered to be outdated if its timestamp difference
    // with the current one exceeds some pre-defined value
    void revise_motion_estimation_list();

    ParticleDistribution m_ParticleDistribution;
    boost::mt19937 m_ParticleDistributionRng;     // random number generator
    bicv_HeadPoseMixtureDynamicModel * m_DynamicModel;
    boost::mt19937 m_DynamicModelRng;             // random number generator
    const Likelihood * m_Likelihood;

    // prior distribution on head scale
    boost::math::normal_distribution<float> m_ScaleAverageBasedPrior;

    // keeps a number of last associated face detections to establish
    // prior distributions over the state
    boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor>
        m_LastAssociatedFaceDetections;

    int m_Id;

    // image providers
    ImageProcessing::ip_ImageProvider * m_DataProvider;
    ImageProcessing::ip_ImageProvider * m_GrayscaleProvider;
    ImageProcessing::ip_GradientImageProcessor * m_GradientProvider;
    ImageProcessing::ip_IntegralGradientHistogramProcessor * m_IntegralHogProvider;
    ImageProcessing::ip_SkinColourProcessor * m_SkinColourProvider;
    ImageProcessing::ip_Dense2DMotionProcessorInria * m_MotionProcessor;
    FaceColorModel::FaceColorModel * mFaceColourModel; // face colour model
    ImageProcessing::ip_IntegralSkinMaskProcessor * m_IntegralSkinProvider;
    ImageProcessing::ip_ImageProvider * m_DepthProvider;

    // feature producers
    ImageProcessing::ip_HogFeatureProducerRounded * m_HogFeatureProducer;
    ImageProcessing::ip_SkinFeatureProducer * m_SkinFeatureProducer;

    // motion processor components
    CMotion2DModel * m_MotionModel;
    CMotion2DEstimator * m_MotionEstimator;
    // list of timestamped motion estimations
    std::list<TimedMotionEstim> m_MotionEstimList;

    // skin colour producer
    ImageProcessing::ip_ColourSkinFeatureProducer * mSkinColourProducer;
    bicv_HeadPoseSkinModel * m_TrainedSkinModel;

    // head pose domain
    const OpenCvPlus::cvp_HeadPoseDiscreteDomain& m_HeadPoseDomain;
    // face detection statistics on head pose and bbox
    const OpenCvPlus::cvp_FaceDetectorStatisticsStorage *
        m_FaceDetectorStatisticsStorage;

    boost::circular_buffer<float> m_LhoodValues;


    // KLUDGE: logging for trackers
    std::ofstream m_Dumpfile;
    int m_IterationCount;
    // END_KLUDGE

    int m_FaceIdx; // Id of the face associated with in detector list

};

} // namespace BICV

#endif // __BICV_HEADPOSETRACKER_H__
