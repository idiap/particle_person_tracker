/**
 * @file cxx/bayes_image/src/bicv_CsMixtureDynamicModel.cc
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
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // ROI converter

// LOCAL INCLUDES
#include <bayes_image/bicv_CsMixtureDynamicModel.h> // declaration of this

// prior statistics on tracker state
static const float MAX_TIME_DELAY_S = 1.0;
static const float MIN_TRANSLATION_X_STDDEV = 5.0;
static const float MAX_TRANSLATION_X_STDDEV = 20.0;
static const float MIN_TRANSLATION_Y_STDDEV = 5.0;
static const float MAX_TRANSLATION_Y_STDDEV = 20.0;
static const float MIN_SCALE_STDDEV = 0.1f;
static const float MAX_SCALE_STDDEV = 1.0f;
static const float MIN_EXCENTRICITY_STDDEV = 0.001f;
static const float MAX_EXCENTRICITY_STDDEV = 0.01f;

static const float FACE_TRANSLATION_X_STDDEV = 10.0;
static const float FACE_TRANSLATION_Y_STDDEV = 10.0;
static const float FACE_SCALE_STDDEV = 0.1f;
static const float FACE_EXCENTRICITY_STDDEV = 0.001f;

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

bicv_CsMixtureDynamicModel::bicv_CsMixtureDynamicModel(
        const BICV::bicv_HeadPoseMixtureDynamicModelConfig& config,
        unsigned num_poses) :
            m_Config(config), m_NumPoses(num_poses) {

    // minimal standard deviation
    m_MinStddevParams.m_TranslationX = MIN_TRANSLATION_X_STDDEV;
    m_MinStddevParams.m_TranslationY = MIN_TRANSLATION_Y_STDDEV;
    m_MinStddevParams.m_Scale = MIN_SCALE_STDDEV;
    m_MinStddevParams.m_Excentricity = MIN_EXCENTRICITY_STDDEV;

    // maximum standard deviation
    m_MaxStddevParams.m_TranslationX = MAX_TRANSLATION_X_STDDEV;
    m_MaxStddevParams.m_TranslationY = MAX_TRANSLATION_Y_STDDEV;
    m_MaxStddevParams.m_Scale = MAX_SCALE_STDDEV;
    m_MaxStddevParams.m_Excentricity = MAX_EXCENTRICITY_STDDEV;

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

} // bicv_CsMixtureDynamicModel

/* virtual */
bicv_CsMixtureDynamicModel::~bicv_CsMixtureDynamicModel() {

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

} // ~bicv_CsMixtureDynamicModel

void
bicv_CsMixtureDynamicModel::prepare(
        const boost::posix_time::ptime& sampling_time,
        const boost::posix_time::ptime& previous_sampling_time,
        const std::vector<double>& motion_parameters,
        const std::vector<bool>& motion_parameters_mask,
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& associated_faces) {

    // derive standard deviation of parameters based on time difference
    bicv_TrackerParameters stddev_params =
            derive_stddev_parameters(sampling_time, previous_sampling_time);

    typedef pair<distribution_type*, distribution_mixture_type::value_type>
        weighted_distribution;
    vector<weighted_distribution>& weighted_distributions =
        m_DynamicModel->elements();

    // set random search standard deviation
    bicv_GeneralDynamicModel * rsDynamicModel =
            dynamic_cast<bicv_GeneralDynamicModel *>(
                    weighted_distributions[RS_DMODEL_IDX].first);
    bicv_GeneralDynamicModelParameters rs_dyn_model_params =
            rsDynamicModel->parameters();
    rs_dyn_model_params.m_StdDev = stddev_params;
    rsDynamicModel->parameters(rs_dyn_model_params);

    // set auto regressive model standard deviation
    bicv_GeneralDynamicModel * arDynamicModel =
            dynamic_cast<bicv_GeneralDynamicModel *>(
                    weighted_distributions[AR_DMODEL_IDX].first);
    bicv_GeneralDynamicModelParameters ar_dyn_model_params =
            arDynamicModel->parameters();
    ar_dyn_model_params.m_StdDev = stddev_params;
    arDynamicModel->parameters(ar_dyn_model_params);

    // set face-based sampling distribution
    if (!associated_faces.empty()) {
        // if faces were detected, derive parameters
        bicv_GeneralDynamicModel * fbDynamicModel =
                dynamic_cast<bicv_GeneralDynamicModel *>(
                        weighted_distributions[FB_DMODEL_IDX].first);
        bicv_GeneralDynamicModelParameters fb_dyn_model_params =
                derive_parameters_from_face_detections(fbDynamicModel,
                        associated_faces);
        fbDynamicModel->parameters(fb_dyn_model_params);
        weighted_distributions[FB_DMODEL_IDX].second = m_Config.m_FBWeight;
    } else {
        // set weight to 0 otherwise
        weighted_distributions[FB_DMODEL_IDX].second = 0;
    }

    // set motion-based sampling distribution
    bicv_GeneralDynamicModel * mbDynamicModel =
            dynamic_cast<bicv_GeneralDynamicModel *>(
                    weighted_distributions[MB_DMODEL_IDX].first);
    bicv_GeneralDynamicModelParameters mb_dyn_model_params =
            derive_parameters_from_motion(
                    mbDynamicModel, motion_parameters, motion_parameters_mask);
    mbDynamicModel->parameters(mb_dyn_model_params);

} // prepare

/* virtual */ bicv_CsMixtureDynamicModel::distribution_type::observation_type
bicv_CsMixtureDynamicModel::sample(rng_engine_type& rng,
        const state_type& state) const {
    return m_DynamicModel->sample(rng, state);
} // sample

/* virtual */ bicv_CsMixtureDynamicModel::distribution_type::value_type
bicv_CsMixtureDynamicModel::evaluate(const state_type& state,
        const observation_type& obs) const {
    return m_DynamicModel->evaluate(state, obs);
} // evaluate

/////////////////////////////// PRIVATE //////////////////////////////////////

bicv_GeneralDynamicModel *
bicv_CsMixtureDynamicModel::create_AR_dynamic_model() {

    bicv_GeneralDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = -1;
    dyn_model_params.m_ArPrev.m_TranslationY = -1;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 2;
    dyn_model_params.m_ArCur.m_TranslationY = 2;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;

    return new bicv_GeneralDynamicModel(dyn_model_params, m_NumPoses);
} // create_AR_dynamic_model

bicv_GeneralDynamicModel *
bicv_CsMixtureDynamicModel::create_MB_dynamic_model() {

    bicv_GeneralDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 1;
    dyn_model_params.m_ArCur.m_TranslationY = 1;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;

    return new bicv_GeneralDynamicModel(dyn_model_params, m_NumPoses);
} // create_MB_dynamic_model

bicv_GeneralDynamicModel *
bicv_CsMixtureDynamicModel::create_FB_dynamic_model() {

    bicv_GeneralDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 0;
    dyn_model_params.m_ArCur.m_TranslationY = 0;
    dyn_model_params.m_ArCur.m_Scale = 0;
    dyn_model_params.m_ArCur.m_Excentricity = 0;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;

    return new bicv_GeneralDynamicModel(dyn_model_params, m_NumPoses);
} // create_FB_dynamic_model

BICV::bicv_GeneralDynamicModel *
bicv_CsMixtureDynamicModel::create_RS_dynamic_model() {

    bicv_GeneralDynamicModelParameters dyn_model_params;

    dyn_model_params.m_StdDev = m_MinStddevParams;

    dyn_model_params.m_ArPrev.m_TranslationX = 0;
    dyn_model_params.m_ArPrev.m_TranslationY = 0;
    dyn_model_params.m_ArPrev.m_Scale = 0;
    dyn_model_params.m_ArPrev.m_Excentricity = 0;

    dyn_model_params.m_ArCur.m_TranslationX = 1;
    dyn_model_params.m_ArCur.m_TranslationY = 1;
    dyn_model_params.m_ArCur.m_Scale = 1;
    dyn_model_params.m_ArCur.m_Excentricity = 1;

    dyn_model_params.m_MotConst.m_TranslationX = 0;
    dyn_model_params.m_MotConst.m_TranslationY = 0;
    dyn_model_params.m_MotConst.m_Scale = 0;
    dyn_model_params.m_MotConst.m_Excentricity = 0;

    return new bicv_GeneralDynamicModel(dyn_model_params, m_NumPoses);
} // create_RS_dynamic_model

bicv_TrackerParameters
bicv_CsMixtureDynamicModel::derive_stddev_parameters(
        const boost::posix_time::ptime& sampling_time,
        const boost::posix_time::ptime& previous_sampling_time) {

    boost::posix_time::time_duration dt =
            sampling_time - previous_sampling_time;

    const long dt_microsec = dt.total_microseconds();
    float dt_microsec_real = static_cast<float>(dt_microsec);

    if (dt_microsec_real > MAX_TIME_DELAY_S) {
        dt_microsec_real = MAX_TIME_DELAY_S;
    }
    bicv_TrackerParameters stddev_params = m_MinStddevParams +
            (m_MaxStddevParams - m_MinStddevParams) *
            (dt_microsec_real / 1.0e6);

    return stddev_params;

} // derive_stddev_parameters

bicv_GeneralDynamicModelParameters
bicv_CsMixtureDynamicModel::derive_parameters_from_face_detections(
        BICV::bicv_GeneralDynamicModel * fb_dynamic_model,
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

    ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(
            chosen_face_detection.m_FaceRegion);
    bicv_TrackerParameters face_params =
            HpParams2RoiConverter::roi2params(face_roi);

    bicv_GeneralDynamicModelParameters dyn_model_params =
            fb_dynamic_model->parameters();

    // fill constant part
    dyn_model_params.m_MotConst = face_params;

    // fill stddev part
    dyn_model_params.m_StdDev.m_TranslationX = FACE_TRANSLATION_X_STDDEV;
    dyn_model_params.m_StdDev.m_TranslationY = FACE_TRANSLATION_Y_STDDEV;
    dyn_model_params.m_StdDev.m_Scale = FACE_SCALE_STDDEV;
    dyn_model_params.m_StdDev.m_Excentricity = FACE_EXCENTRICITY_STDDEV;

    return dyn_model_params;

} // derive_dynamics_from_face_detection

bicv_GeneralDynamicModelParameters
bicv_CsMixtureDynamicModel::derive_parameters_from_motion(
        BICV::bicv_GeneralDynamicModel * mb_dynamic_model,
        const vector<double>& motion_parameters,
        const vector<bool>& motion_parameters_mask) {

    static const unsigned TRANSLATION_X_IDX = 0;
    static const unsigned TRANSLATION_Y_IDX = 1;
    static const unsigned TRANSLATION_DIV_X_IDX = 2;

    bicv_GeneralDynamicModelParameters dyn_model_params =
            mb_dynamic_model->parameters();

    if (motion_parameters_mask[TRANSLATION_X_IDX]) {
        dyn_model_params.m_MotConst.m_TranslationX =
                motion_parameters[TRANSLATION_X_IDX];
    } else {
        dyn_model_params.m_MotConst.m_TranslationX = 0;
    }

    if (motion_parameters_mask[TRANSLATION_Y_IDX]) {
        dyn_model_params.m_MotConst.m_TranslationY =
                motion_parameters[TRANSLATION_Y_IDX];
    } else {
        dyn_model_params.m_MotConst.m_TranslationY = 0;
    }

    if (motion_parameters_mask[TRANSLATION_DIV_X_IDX]) {
        dyn_model_params.m_ArCur.m_Scale =
                1 + motion_parameters[TRANSLATION_DIV_X_IDX];
    } else {
        dyn_model_params.m_ArCur.m_Scale = 1;
    }

    return dyn_model_params;

} // derive_parameters_from_motion

} // namespace BICV
