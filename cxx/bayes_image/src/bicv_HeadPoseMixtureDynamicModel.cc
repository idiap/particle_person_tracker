/**
 * @file cxx/bayes_image/src/bicv_HeadPoseMixtureDynamicModel.cc
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

// SYSTEM INCLUDES
#include <utility>                                        // STL pair

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>                // ROI window

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseMixtureDynamicModel.h> // declaration of this
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>

// prior statistics on tracker state
static const float MAX_TIME_DELAY_S = 1.0;
static const float MIN_TRANSLATION_X_STDDEV = 5.0;
static const float MAX_TRANSLATION_X_STDDEV = 50.0;
static const float MIN_TRANSLATION_Y_STDDEV = 5.0;
static const float MAX_TRANSLATION_Y_STDDEV = 50.0;
static const float MIN_SCALE_STDDEV = 0.1f;
static const float MAX_SCALE_STDDEV = 1.0f;
static const float MIN_EXCENTRICITY_STDDEV = 0.001f;
static const float MAX_EXCENTRICITY_STDDEV = 0.01f;
static const float MIN_PAN_STDDEV = 8.0;
static const float MAX_PAN_STDDEV = 20.0f;
static const float MIN_TILT_STDDEV = 8.0f;
static const float MAX_TILT_STDDEV = 20.0f;
static const float MIN_ROLL_STDDEV = 5.0f;
static const float MAX_ROLL_STDDEV = 10.0f;

using namespace std;
using namespace BayesFilter;
using namespace OpenCvPlus;
using namespace ImageProcessing;

//////////////////////////// LOCAL CONSTANTS /////////////////////////////////
// index of random search model in the mixture model
static const unsigned RS_DMODEL_IDX = 0;
// index of autoregressive model in the mixture model
static const unsigned AR_DMODEL_IDX = 1;
// index of face-based dynamics model in the mixture model
static const unsigned FB_DMODEL_IDX = 2;
// index of motion-based dynamics model in the mixture model
static const unsigned MB_DMODEL_IDX = 3;
// number of dynamic models in the mixture
static const unsigned NUM_DMODELS = 4;

namespace BICV {

/////////////////////////////// PUBLIC ///////////////////////////////////////

bicv_HeadPoseMixtureDynamicModel::bicv_HeadPoseMixtureDynamicModel(
            const bicv_HeadPoseMixtureDynamicModelConfig& config,
            const cvp_HeadPoseDiscreteDomain& head_pose_domain,
            const cvp_FaceDetectorStatisticsStorage& face_detector_stats) :
            m_HeadPoseDomain(head_pose_domain),
            m_FaceDetectorStatisticsStorage(face_detector_stats),
            m_Config(config) {

    // minimal standard deviation
    m_MinStddevParams.m_TranslationX = MIN_TRANSLATION_X_STDDEV;
    m_MinStddevParams.m_TranslationY = MIN_TRANSLATION_Y_STDDEV;
    m_MinStddevParams.m_Scale = MIN_SCALE_STDDEV;
    m_MinStddevParams.m_Excentricity = MIN_EXCENTRICITY_STDDEV;
    m_MinStddevParams.m_HeadPose.pan(MIN_PAN_STDDEV);
    m_MinStddevParams.m_HeadPose.tilt(MIN_TILT_STDDEV);
    m_MinStddevParams.m_HeadPose.roll(MIN_ROLL_STDDEV);

    // maximum standard deviation
    m_MaxStddevParams.m_TranslationX = MAX_TRANSLATION_X_STDDEV;
    m_MaxStddevParams.m_TranslationY = MAX_TRANSLATION_Y_STDDEV;
    m_MaxStddevParams.m_Scale = MAX_SCALE_STDDEV;
    m_MaxStddevParams.m_Excentricity = MAX_EXCENTRICITY_STDDEV;
    m_MaxStddevParams.m_HeadPose.pan(MAX_PAN_STDDEV);
    m_MaxStddevParams.m_HeadPose.tilt(MAX_TILT_STDDEV);
    m_MaxStddevParams.m_HeadPose.roll(MAX_ROLL_STDDEV);

    // create dynamic models and add them to the mixture model
    vector<pair<distribution_type *,
        distribution_mixture_type::value_type> > weighted_distributions(
                NUM_DMODELS);
    weighted_distributions[RS_DMODEL_IDX].first = create_RS_dynamic_model();
    weighted_distributions[RS_DMODEL_IDX].second = config.m_RSWeight;
    weighted_distributions[AR_DMODEL_IDX].first = create_AR_dynamic_model();
    weighted_distributions[AR_DMODEL_IDX].second = config.m_ARWeight;
    weighted_distributions[FB_DMODEL_IDX].first = create_FB_dynamic_model();
    weighted_distributions[FB_DMODEL_IDX].second = config.m_FBWeight;
    weighted_distributions[MB_DMODEL_IDX].first = create_MB_dynamic_model();
    weighted_distributions[MB_DMODEL_IDX].second = config.m_MBWeight;
    m_DynamicModel = new distribution_mixture_type(weighted_distributions);

} // bicv_HeadPoseMixtureDynamicModel

/* virtual */
bicv_HeadPoseMixtureDynamicModel::~bicv_HeadPoseMixtureDynamicModel() {

    // delete individual dynamic models
    typedef pair<distribution_type*, distribution_mixture_type::value_type>
        weighted_distribution;
    vector<weighted_distribution>& weighted_distributions =
        m_DynamicModel->elements();
    BOOST_FOREACH(weighted_distribution wdist, weighted_distributions) {
        delete wdist.first;
    }

    // delete dynamic mixture model
    delete m_DynamicModel;
    m_DynamicModel = 0;

} // ~bicv_HeadPoseMixtureDynamicModel

void
bicv_HeadPoseMixtureDynamicModel::prepare(
        const boost::posix_time::ptime& sampling_time,
        const boost::posix_time::ptime& previous_sampling_time,
        const ImageProcessing::ip_MotionParameters& motion_parameters,
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& associated_faces) {

    // derive standard deviation of parameters based on time difference
    bicv_HeadPoseParameters stddev_params = derive_stddev_parameters(
            sampling_time, previous_sampling_time);

    typedef pair<distribution_type*,
                 distribution_mixture_type::value_type>
      weighted_distribution;

    vector<weighted_distribution>& weighted_distributions =
        m_DynamicModel->elements();

    // set random search standard deviation
    bicv_ARHeadPoseDynamicModel * rsDynamicModel =
            dynamic_cast<bicv_ARHeadPoseDynamicModel *>(
                    weighted_distributions[RS_DMODEL_IDX].first);
    bicv_ARHeadPoseDynamicModelParameters rs_dyn_model_params =
            rsDynamicModel->parameters();
    rs_dyn_model_params.m_StdDev = stddev_params;
    rsDynamicModel->parameters(rs_dyn_model_params);

    // set auto regressive model standard deviation
    bicv_ARHeadPoseDynamicModel * arDynamicModel =
            dynamic_cast<bicv_ARHeadPoseDynamicModel *>(
                    weighted_distributions[AR_DMODEL_IDX].first);
    bicv_ARHeadPoseDynamicModelParameters ar_dyn_model_params =
            arDynamicModel->parameters();
    ar_dyn_model_params.m_StdDev = stddev_params;
    arDynamicModel->parameters(ar_dyn_model_params);

    // set face-based sampling distribution
    if (!associated_faces.empty()) {
        // if faces were detected, derive parameters
        bicv_ARHeadPoseDynamicModel * fbDynamicModel =
                dynamic_cast<bicv_ARHeadPoseDynamicModel *>(
                        weighted_distributions[FB_DMODEL_IDX].first);
        bicv_ARHeadPoseDynamicModelParameters fb_dyn_model_params =
                derive_parameters_from_face_detections(fbDynamicModel,
                                                       associated_faces);
        fbDynamicModel->parameters(fb_dyn_model_params);
        weighted_distributions[FB_DMODEL_IDX].second = m_Config.m_FBWeight;
    } else {
        // set weight to 0 otherwise
        weighted_distributions[FB_DMODEL_IDX].second = 0;
    }

    // set motion-based sampling distribution
    bicv_ARHeadPoseDynamicModel * mbDynamicModel =
            dynamic_cast<bicv_ARHeadPoseDynamicModel *>(
                    weighted_distributions[MB_DMODEL_IDX].first);
    bicv_ARHeadPoseDynamicModelParameters mb_dyn_model_params =
            derive_parameters_from_motion(mbDynamicModel, motion_parameters);
    mbDynamicModel->parameters(mb_dyn_model_params);

} // prepare

/* virtual */ bicv_HeadPoseMixtureDynamicModel::distribution_type::observation_type
bicv_HeadPoseMixtureDynamicModel::sample(rng_engine_type& rng,
        const state_type& state) const {
    return m_DynamicModel->sample(rng, state);
} // sample

/* virtual */ bicv_HeadPoseMixtureDynamicModel::distribution_type::value_type
bicv_HeadPoseMixtureDynamicModel::evaluate(const state_type& state,
        const observation_type& obs) const {
    return m_DynamicModel->evaluate(state, obs);
} // evaluate

bicv_HeadPoseMixtureDynamicModel::distribution_type::value_type
bicv_HeadPoseMixtureDynamicModel::evaluate_dynamics(
        const state_type& state_old, const state_type& state_new) const {

    const vector<pair<distribution_type *,
        distribution_mixture_type::value_type> > &
        weighted_distributions = m_DynamicModel->elements();

    distribution_type::value_type weight_rs =
            weighted_distributions[RS_DMODEL_IDX].second;
    distribution_type::value_type weight_ar =
            weighted_distributions[AR_DMODEL_IDX].second;

    distribution_type * distr_rs = weighted_distributions[RS_DMODEL_IDX].first;
    distribution_type * distr_ar = weighted_distributions[AR_DMODEL_IDX].first;

    distribution_type::value_type value =
            (weight_rs * distr_rs->evaluate(state_old, state_new) +
             weight_ar * distr_ar->evaluate(state_old, state_new)) /
             (weight_rs + weight_ar);

    return value;

} // evaluate_dynamics

bicv_HeadPoseMixtureDynamicModel::distribution_type::value_type
bicv_HeadPoseMixtureDynamicModel::evaluate_proposal(
        const state_type& state_old, const state_type& state_new) const {

    typedef pair<distribution_type *, distribution_mixture_type::value_type>
        WeightedDistribution;
    const vector<WeightedDistribution> &
        weighted_distributions = m_DynamicModel->elements();

    distribution_type::value_type value = 0;
    distribution_type::value_type sum_weights = 0;

    BOOST_FOREACH(const WeightedDistribution& wdistr, weighted_distributions) {
        value += wdistr.second * wdistr.first->evaluate(state_old, state_new);
        sum_weights += wdistr.second;
    }
    return value;

} // evaluate_proposal

/////////////////////////////// PRIVATE //////////////////////////////////////

bicv_ARHeadPoseDynamicModel *
bicv_HeadPoseMixtureDynamicModel::create_AR_dynamic_model() {

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = -1;
    dyn_model_params.m_ArPrev.m_TranslationY = -1;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;
    dyn_model_params.m_ArPrev.m_HeadPose = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 2;
    dyn_model_params.m_ArCur.m_TranslationY = 2;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;
    dyn_model_params.m_ArCur.m_HeadPose = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;
    dyn_model_params.m_MotConst.m_HeadPose = 0;

    return new bicv_ARHeadPoseDynamicModel(dyn_model_params, m_HeadPoseDomain);
} // create_AR_dynamic_model

bicv_ARHeadPoseDynamicModel *
bicv_HeadPoseMixtureDynamicModel::create_MB_dynamic_model() {

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;
    dyn_model_params.m_ArPrev.m_HeadPose = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 1;
    dyn_model_params.m_ArCur.m_TranslationY = 1;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;
    dyn_model_params.m_ArCur.m_HeadPose = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;
    dyn_model_params.m_MotConst.m_HeadPose = 0;

    return new bicv_ARHeadPoseDynamicModel(dyn_model_params, m_HeadPoseDomain);
} // create_MB_dynamic_model

bicv_ARHeadPoseDynamicModel *
bicv_HeadPoseMixtureDynamicModel::create_FB_dynamic_model() {

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;
    dyn_model_params.m_ArPrev.m_HeadPose = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 0;
    dyn_model_params.m_ArCur.m_TranslationY = 0;
    dyn_model_params.m_ArCur.m_Scale = 0;
    dyn_model_params.m_ArCur.m_Excentricity = 0;
    dyn_model_params.m_ArCur.m_HeadPose = 0;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;
    dyn_model_params.m_MotConst.m_HeadPose = 0;

    return new bicv_ARHeadPoseDynamicModel(dyn_model_params, m_HeadPoseDomain);
} // create_FB_dynamic_model

BICV::bicv_ARHeadPoseDynamicModel *
bicv_HeadPoseMixtureDynamicModel::create_RS_dynamic_model() {

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;
    dyn_model_params.m_ArPrev.m_HeadPose = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 1;
    dyn_model_params.m_ArCur.m_TranslationY = 1;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;
    dyn_model_params.m_ArCur.m_HeadPose = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;
    dyn_model_params.m_MotConst.m_HeadPose = 0;

    return new bicv_ARHeadPoseDynamicModel(dyn_model_params, m_HeadPoseDomain);
} // create_RS_dynamic_model

bicv_HeadPoseParameters
bicv_HeadPoseMixtureDynamicModel::derive_stddev_parameters(
        const boost::posix_time::ptime& sampling_time,
        const boost::posix_time::ptime& previous_sampling_time) {

    boost::posix_time::time_duration dt =
            sampling_time - previous_sampling_time;

    const long dt_microsec = dt.total_microseconds();
    float dt_microsec_real = static_cast<float>(dt_microsec);

    if (dt_microsec_real > MAX_TIME_DELAY_S) {
        dt_microsec_real = MAX_TIME_DELAY_S;
    }
    bicv_HeadPoseParameters stddev_params = m_MinStddevParams +
            (m_MaxStddevParams - m_MinStddevParams) *
            (dt_microsec_real / 1.0e6);

    return stddev_params;

} // derive_stddev_parameters

bicv_ARHeadPoseDynamicModelParameters
bicv_HeadPoseMixtureDynamicModel::derive_parameters_from_face_detections(
        BICV::bicv_ARHeadPoseDynamicModel * fb_dynamic_model,
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& face_detections) {

    OpenCvPlus::cvp_FaceDescriptor chosen_face_detection;

    // check if face detections contain frontal face
    bool found_frontal_face_detection = false;
    BOOST_FOREACH(const cvp_FaceDescriptor& face_detection, face_detections) {
        if (face_detection.m_Pose == CVP_FACEDETECTOR_FACE) {
            found_frontal_face_detection = true;
            chosen_face_detection = face_detection;
            break;
        }
    }

    if (!found_frontal_face_detection) {
        chosen_face_detection = face_detections.front();
    }

    const cvp_FaceDetectorStatistics& fd_stats =
            m_FaceDetectorStatisticsStorage.face_detector_statistics(
                    chosen_face_detection.m_Pose);
    ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(
            chosen_face_detection.m_FaceRegion);
    bicv_HeadPoseParameters face_params =
            HpParams2RoiConverter::roi2hpparams(face_roi);

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params =
            fb_dynamic_model->parameters();

    // fill constant part
    dyn_model_params.m_MotConst.m_TranslationX =
            face_params.m_TranslationX + fd_stats.m_MeanBBoxTransform.val[0];
    dyn_model_params.m_MotConst.m_TranslationY =
            face_params.m_TranslationY + fd_stats.m_MeanBBoxTransform.val[1];
    dyn_model_params.m_MotConst.m_Scale =
            face_params.m_Scale * fd_stats.m_MeanBBoxTransform.val[2];
    dyn_model_params.m_MotConst.m_Excentricity =
            face_params.m_Excentricity * fd_stats.m_MeanBBoxTransform.val[3] /
            fd_stats.m_MeanBBoxTransform.val[2];
    dyn_model_params.m_MotConst.m_HeadPose.pan(fd_stats.m_MeanHeadPose.val[0]);
    dyn_model_params.m_MotConst.m_HeadPose.tilt(fd_stats.m_MeanHeadPose.val[1]);
    dyn_model_params.m_MotConst.m_HeadPose.roll(fd_stats.m_MeanHeadPose.val[2]);

    // fill stddev part
    dyn_model_params.m_StdDev.m_TranslationX =
            fd_stats.m_StddevBBoxTransform.val[0];
    dyn_model_params.m_StdDev.m_TranslationY =
            fd_stats.m_StddevBBoxTransform.val[1];
    dyn_model_params.m_StdDev.m_Scale =
            max<double>(
                face_params.m_Scale * fd_stats.m_StddevBBoxTransform.val[2],
                MIN_SCALE_STDDEV);

    // approximate formula (!):
//    dyn_model_params.m_StdDev.m_Excentricity =
//            face_params.m_Excentricity * fd_stats.m_StddevBBoxTransform.val[3];
    dyn_model_params.m_StdDev.m_Excentricity = MAX_EXCENTRICITY_STDDEV;
    dyn_model_params.m_StdDev.m_HeadPose.pan(fd_stats.m_StddevHeadPose.val[0]);
    dyn_model_params.m_StdDev.m_HeadPose.tilt(fd_stats.m_StddevHeadPose.val[1]);
    dyn_model_params.m_StdDev.m_HeadPose.roll(fd_stats.m_StddevHeadPose.val[2]);

    return dyn_model_params;

} // derive_dynamics_from_face_detection

bicv_ARHeadPoseDynamicModelParameters
bicv_HeadPoseMixtureDynamicModel::derive_parameters_from_motion(
        bicv_ARHeadPoseDynamicModel * mb_dynamic_model,
        const ip_MotionParameters& motion_parameters) {

    bicv_ARHeadPoseDynamicModelParameters dyn_model_params =
            mb_dynamic_model->parameters();

    if (motion_parameters.m_Flags[ip_MotionParameters::TRANSLATION_X_IDX]) {
        dyn_model_params.m_MotConst.m_TranslationX =
               motion_parameters.m_Parameters[
                   ip_MotionParameters::TRANSLATION_X_IDX];
    } else {
        dyn_model_params.m_MotConst.m_TranslationX = 0;
    }

    if (motion_parameters.m_Flags[ip_MotionParameters::TRANSLATION_Y_IDX]) {
        dyn_model_params.m_MotConst.m_TranslationY =
               motion_parameters.m_Parameters[
                   ip_MotionParameters::TRANSLATION_Y_IDX];
    } else {
        dyn_model_params.m_MotConst.m_TranslationY = 0;
    }

    if (motion_parameters.m_Flags[ip_MotionParameters::DIV_X_IDX]) {
        dyn_model_params.m_ArCur.m_Scale =
               1 + motion_parameters.m_Parameters[
                   ip_MotionParameters::DIV_X_IDX];
    } else {
        dyn_model_params.m_ArCur.m_Scale = 1;
    }

    return dyn_model_params;

} // derive_parameters_from_motion

} // namespace BICV
