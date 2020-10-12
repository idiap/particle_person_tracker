// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ColourSegmentationTracker - class to track colour-based segmentation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <bayes_image/bicv_ColourSegmentationTracker.h>         // declaration of this
#include <bayes_image/bicv_StateImageLikelihood_FCM.h>          // likelihood
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // ROI converter

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                       // foreach loop
#include <boost/random.hpp>                        // taus88 RNG
#include <boost/math/distributions/normal.hpp>     // normal distribution
#include <ctime>                                   // time
#include <list>                                    // STL list
#include <sstream>                                 // STL string stream
#include <numeric>                                 // accumulate
#include <iterator>                                // debugging purposes

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;

// Dumps tracker particles
//#define ENABLE_TRACKER_DUMP

// Updates face colour model when face is detected
#define UPDATE_FACE_COLOUR_MODEL

#ifdef ENABLE_TRACKER_DUMP
static int tracker_counter = 1;
int GLOBAL_FRAME_COUNTER = 1;
#endif

//////////////////////////// LOCAL CONSTANTS /////////////////////////////////

// Gaussian distribution prior parameters for SCALE component
// Gaussian distribution prior parameters for SCALE component
static const float SCALE_PRIOR_MEAN = 1.0;
static const float SCALE_PRIOR_STDDEV = 0.5;
static const float SCALE_PRIOR_MIN_STDDEV = 0.1;
static const float SCALE_WEIGHT_CORRECTION_MIN = 0.1;

// Face observation standard deviations
static const float FACE_TRANSLATION_X_STDDEV = 10.0;
static const float FACE_TRANSLATION_Y_STDDEV = 10.0;
static const float FACE_SCALE_STDDEV = 0.1f;
static const float FACE_EXCENTRICITY_STDDEV = 0.001f;

static const int LIKELIHOOD_VALUES_COUNT = 20;
static const float LIKELIHOOD_TRACKING_THRESHOLD = 5e-5;

static const unsigned NUM_FACE_DETECTIONS_TO_KEEP = 10;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static int GLOBAL_TRACKER_COUNTER = 2;

/////////////////////////////// PUBLIC ///////////////////////////////////////

namespace BICV {

bicv_ColourSegmentationTracker::bicv_ColourSegmentationTracker(
        bicv_CsMixtureDynamicModel * dyn_model,
        ImageProcessing::ip_ImageProvider * data_provider,
        ImageProcessing::ip_Dense2DMotionProcessorInria * motion_processor,
        FaceColorModel::FaceColorModel * face_colour_model,
        unsigned num_particles) :

    m_ParticleDistribution(bicv_TrackerState(), num_particles),
    m_ParticleDistributionRng(generate_random_seed()),
    m_DynamicModel(dyn_model),
    m_DynamicModelRng(generate_random_seed()),
    m_MotionProcessor(motion_processor),
    m_ScaleAverageBasedPrior(SCALE_PRIOR_MEAN, SCALE_PRIOR_STDDEV),
    m_LastAssociatedFaceDetections(NUM_FACE_DETECTIONS_TO_KEEP),
    m_DataProvider(data_provider),
    m_FaceColourModel(face_colour_model),
    m_LhoodValues(LIKELIHOOD_VALUES_COUNT, 2 * LIKELIHOOD_TRACKING_THRESHOLD) {

    m_Id = GLOBAL_TRACKER_COUNTER++;

    m_Likelihood = new bicv_StateImageLikelihood_FCM(
            face_colour_model);

    // initialize motion estimator parameters
    m_MotionModel = new CMotion2DModel();
    m_MotionEstimator = new CMotion2DEstimator();
    ip_Dense2DMotionProcessorInria::apply_config_options(
            m_MotionProcessor->getConfig(), m_MotionModel, m_MotionEstimator);

//    cout << "Tracker, initialized face colour model with " << mFaceColourModel << endl;

#ifdef ENABLE_TRACKER_DUMP
    ostringstream oss;
    oss << "tracker_" << (tracker_counter++) << "_dump.txt";
    m_Dumpfile.open(oss.str().c_str());
    m_IterationCount = 1;
#endif

} // bicv_ColourSegmentationTracker

bicv_ColourSegmentationTracker::~bicv_ColourSegmentationTracker() {
#ifdef ENABLE_TRACKER_DUMP
    m_Dumpfile.close();
#endif
    delete m_Likelihood; m_Likelihood = 0;
    delete m_FaceColourModel; m_FaceColourModel = 0;
} // ~bicv_ColourSegmentationTracker

void bicv_ColourSegmentationTracker::init(
        const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
        const boost::posix_time::ptime & timestamp) {

    // initialize sampling distribution
    boost::normal_distribution<float> gaussian_dist;
    boost::variate_generator<boost::mt19937&,
        boost::normal_distribution<float> > gaussian(m_DynamicModelRng,
        gaussian_dist);

    // convert face descriptor to tracker state
    bicv_TrackerParameters face_params =
            HpParams2RoiConverter::roi2params(
                    ip_RoiWindow::from_CvRect(init_candidate.m_FaceRegion));

    // resample all particles
    vector<ParticleDistribution::element_type>& elements =
        m_ParticleDistribution.elements();

    bicv_TrackerParameters params;
    BOOST_FOREACH(ParticleDistribution::element_type& element, elements) {
        params.m_TranslationX = face_params.m_TranslationX + gaussian() *
                FACE_TRANSLATION_X_STDDEV;
        params.m_TranslationY = face_params.m_TranslationY + gaussian() *
                FACE_TRANSLATION_Y_STDDEV;
        params.m_Scale = face_params.m_Scale + gaussian() *
                FACE_SCALE_STDDEV;
        params.m_Excentricity = face_params.m_Excentricity + gaussian() *
                FACE_EXCENTRICITY_STDDEV;
        params.m_PoseIndex = pose_index_from_face_orientation(
                init_candidate.m_Pose);
        params.m_Time = timestamp;

        element.first.m_ParamsCur = params;
        element.first.m_ParamsPrev = params;
        element.second = 1.0f;
    }

    // save detected face into associated faces
    observe_face(init_candidate);

} // init

void bicv_ColourSegmentationTracker::iterate() {

    invalidate_providers();

    IplImage * source_image = m_DataProvider->image();

    vector<ParticleDistribution::element_type> & elements =
        m_ParticleDistribution.elements();

    // get input image width and height and define image ROI
    const int width = m_DataProvider->image_buffer()->width;
    const int height = m_DataProvider->image_buffer()->height;
    const ip_RoiWindow global_roi(0, 0, width, height);

    // get current and previous timestamps
    const boost::posix_time::ptime current_data_time = m_DataProvider->time();
    const boost::posix_time::ptime previous_data_time =
            elements[0].first.m_ParamsCur.m_Time;

    // estimate motion
    vector<double> motion_parameters(CMotion2DModel::MDL_NMAX_COEF);
    vector<bool> motion_parameters_mask(CMotion2DModel::MDL_NMAX_COEF, false);
    bicv_TrackerState mean_state = mean(particle_distribution());
    ip_RoiWindow mean_roi = HpParams2RoiConverter::params2roi(
            mean_state.m_ParamsCur);
    static const float support_scale = 1.2;
    ip_RoiWindow mean_roi_scaled = scale(mean_roi, support_scale);
    ip_RoiWindow motion_estimation_roi;
    if (intersect(global_roi, mean_roi_scaled, motion_estimation_roi)) {
        m_MotionProcessor->reestimate4roi(motion_estimation_roi,
                m_MotionModel, m_MotionEstimator);
        m_MotionModel->getParameters(&motion_parameters[0]);
        motion_parameters_mask[0] = true;
        motion_parameters_mask[1] = true;
        motion_parameters_mask[2] = false;
    } else {
        motion_parameters_mask[0] = false;
        motion_parameters_mask[1] = false;
        motion_parameters_mask[2] = false;
    }

    // prepare dynamic model for sampling
    list<cvp_FaceDescriptor> associated_face_detections =
            associated_face_detections_for_timestamp(current_data_time);
    m_DynamicModel->prepare(current_data_time, previous_data_time,
            motion_parameters, motion_parameters_mask,
            associated_face_detections);

    // predict particles using the dynamic model
    list<bicv_TrackerState> new_particles;
    for (unsigned i = 0; i < elements.size(); ++i) {
        bicv_TrackerState particle =
            sample(m_ParticleDistributionRng, m_ParticleDistribution);
        bicv_TrackerState propagated_particle =
                m_DynamicModel->sample(m_DynamicModelRng, particle);
//        bicv_TrackerState corrected_particle = correct_state(
//                propagated_particle);
//        new_particles.push_back(corrected_particle);
        new_particles.push_back(propagated_particle);
    }

#ifdef ENABLE_TRACKER_DUMP
    m_Dumpfile << "frame " << GLOBAL_FRAME_COUNTER << endl;
#endif

    // compute the bounding box for all the particles
    int first_col = numeric_limits<int>::max();
    int last_col = numeric_limits<int>::min();
    int first_row = numeric_limits<int>::max();
    int last_row = numeric_limits<int>::min();
    BOOST_FOREACH(const bicv_TrackerState& particle, new_particles) {
        ip_RoiWindow roi = HpParams2RoiConverter::params2roi(
                particle.m_ParamsCur);
        if (first_col > roi.m_iFirstColumn) {
            first_col = roi.m_iFirstColumn;
        }
        if (last_col < roi.m_iFirstColumn + roi.m_iWidth - 1) {
            last_col = roi.m_iFirstColumn + roi.m_iWidth - 1;
        }
        if (first_row > roi.m_iFirstRow) {
            first_row = roi.m_iFirstRow;
        }
        if (last_row < roi.m_iFirstRow + roi.m_iHeight - 1) {
            last_row = roi.m_iFirstRow + roi.m_iHeight - 1;
        }
    }

    ip_RoiWindow particles_roi(first_col, first_row,
            last_col - first_col, last_row - first_row);
    ip_RoiWindow particles_roi_corrected;
    if (!intersect(global_roi, particles_roi, particles_roi_corrected)) {
        return;
    }

    // update skin colour model if face was observed
#ifdef UPDATE_FACE_COLOUR_MODEL
    if (!associated_face_detections.empty()) {
        cvp_FaceDescriptor chosen_face_detection;
        // check if face detections contain frontal face
        bool found_frontal_face_detection = false;
        BOOST_FOREACH(const cvp_FaceDescriptor& face_detection,
                associated_face_detections) {
            if (face_detection.m_Pose == CVP_FACEDETECTOR_FACE) {
                found_frontal_face_detection = true;
                chosen_face_detection = face_detection;
                break;
            }
        }

        if (found_frontal_face_detection) {
            IplImage * source_img = m_DataProvider->image();
            m_FaceColourModel->adapt_to(source_img,
                    chosen_face_detection.m_FaceRegion);
        }
    }
#endif

    m_FaceColourModel->cache_probability_maps(source_image,
        ip_RoiWindow::to_CvRect(particles_roi_corrected));

    // reweight particles by observations
    const list<bicv_TrackerState>::const_iterator first_ii =
        new_particles.begin();
    const list<bicv_TrackerState>::const_iterator last_ii =
        new_particles.end();
    list<bicv_TrackerState>::const_iterator ii = first_ii;

    std::list<float> weights;
    for(; ii != last_ii; ++ii) {
        weights.push_back(m_Likelihood->evaluate(*ii, source_image) *
                weight_correction(*ii));

#ifdef ENABLE_TRACKER_DUMP
        m_Dumpfile << "particle " << (*ii) << ", " <<
            "prior correction (" << weight_correction_prior(*ii) << "), " <<
            "face correction (" << weight_correction_face(*ii) << "), " <<
            "likelihood (" << m_Likelihood->evaluate(*ii, source_image) << "), " <<
            "corrected likelihood (" << m_Likelihood->evaluate(*ii, source_image) *
                weight_correction(*ii) << ")" << endl;
#endif

    }

    // write results to the particle distribution
    list<float>::const_iterator jj = weights.begin();

    vector<ParticleDistribution::element_type>::iterator kk = elements.begin();
    for (ii = first_ii; ii != last_ii; ++ii, ++jj, ++kk) {
        kk->first = *ii;
        kk->second = *jj;
    }

    m_LhoodValues.push_back(internal_tracking_score());

} // iterate

bool bicv_ColourSegmentationTracker::should_continue_tracking() const {
//    return true;
    if (!m_LhoodValues.empty()) {
        float mean_lhood = accumulate(
                m_LhoodValues.begin(), m_LhoodValues.end(), 0.0) /
                m_LhoodValues.size();
        return (mean_lhood > LIKELIHOOD_TRACKING_THRESHOLD);
    } else {
        return true;
    }
} // should_continue_tracking

float bicv_ColourSegmentationTracker::internal_tracking_score() const {
    bicv_TrackerState score_state = mean(m_ParticleDistribution);

    IplImage * source_image = m_DataProvider->image();
    Likelihood::value_type lhood = m_Likelihood->evaluate(score_state,
            source_image);

    //float prior_pose = weight_correction_prior(score_state);
    //float prior_face = weight_correction_face(score_state);

    return lhood;
} // internal_tracking_score

void bicv_ColourSegmentationTracker::observe_face(
        const OpenCvPlus::cvp_FaceDescriptor& face_descriptor) {
    CvRect face_region = face_descriptor.m_FaceRegion;
//    cout << "Observed face: " << ip_RoiWindow::from_CvRect(face_region) << endl;
    if ((face_region.x > 0 ) && (face_region.y > 0) &&
        (face_region.width > 0) && (face_region.height > 0)) {
        m_LastAssociatedFaceDetections.push_back(face_descriptor);
        derive_average_scale_distribution(m_LastAssociatedFaceDetections,
                SCALE_PRIOR_MIN_STDDEV);
    }

} // observe_face

const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor> &
    bicv_ColourSegmentationTracker::associated_face_detections() const {
    return m_LastAssociatedFaceDetections;
} // associated_face_detections

std::list<OpenCvPlus::cvp_FaceDescriptor>
bicv_ColourSegmentationTracker::associated_face_detections_for_timestamp(
            const boost::posix_time::ptime& timestamp,
            const boost::posix_time::time_duration& duration) const {
    std::list<OpenCvPlus::cvp_FaceDescriptor> result_face_detections;
    boost::posix_time::time_duration td;
    BOOST_FOREACH(const cvp_FaceDescriptor& face_detection,
            m_LastAssociatedFaceDetections) {
        td = face_detection.m_DetectionTime - timestamp;
        if (td.is_negative()) {
            td = td.invert_sign();
        }
        if (td <= duration) {
            result_face_detections.push_back(face_detection);
        }
    }
    return result_face_detections;
} // associated_face_detections_for_timestamp

float bicv_ColourSegmentationTracker::mode_likelihood() const {
     bicv_TrackerState mode_state = mode(m_ParticleDistribution);
    IplImage * source_image = m_DataProvider->image();
    return m_Likelihood->evaluate(mode_state, source_image) *
            weight_correction(mode_state);
} // bicv_HeadPoseTracker

void bicv_ColourSegmentationTracker::invalidate_providers() {
} // invalidate_providers

inline
unsigned bicv_ColourSegmentationTracker::generate_random_seed() const {
    static const unsigned NUM_BOOTSTRAP_ITER = 100;
    boost::taus88 rng(static_cast<unsigned>(time(0)));
    unsigned result;
    for (unsigned i = 0; i < NUM_BOOTSTRAP_ITER; ++i) {
        rng();
    }
    return result = rng();
} // generate_random_seed

inline float bicv_ColourSegmentationTracker::weight_correction(
        const bicv_TrackerState& state) const {

    float weight_correction_prior_part = weight_correction_prior(state);
    float weight_correction_face_part = weight_correction_face(state);
    return weight_correction_prior_part * weight_correction_face_part;

} // weight_correction

inline float bicv_ColourSegmentationTracker::weight_correction_prior(
        const bicv_TrackerState& state) const {

    //const float scale = state.m_ParamsCur.m_Scale;
    //const float excentricity = state.m_ParamsCur.m_Excentricity;

    using namespace boost::math;

//    normal_distribution<float> normal_scale_dist(
//            SCALE_PRIOR_MEAN, SCALE_PRIOR_STDDEV);
//    normal_distribution<float> normal_excentricity_dist(
//            EXCENTRICITY_PRIOR_MEAN, EXCENTRICITY_PRIOR_STDDEV);

    float result = 1;

    float correction_term =
        pdf(m_ScaleAverageBasedPrior, state.m_ParamsCur.m_Scale);
    result *= max(correction_term, SCALE_WEIGHT_CORRECTION_MIN);

    return result;
} // weight_correction_prior

inline float bicv_ColourSegmentationTracker::weight_correction_face(
        const bicv_TrackerState& state) const {

    using namespace boost::math;

    float result = 1;
    boost::posix_time::ptime current_data_time = m_DataProvider->time();
    list<cvp_FaceDescriptor> current_face_detections =
        associated_face_detections_for_timestamp(current_data_time);

    if (current_face_detections.empty()) {
        return result;
    }

    OpenCvPlus::cvp_FaceDescriptor chosen_face_detection;

    // check if face detections contain frontal face
    bool found_frontal_face_detection = false;
    BOOST_FOREACH(const cvp_FaceDescriptor& face_detection,
            current_face_detections) {
        if (face_detection.m_Pose == CVP_FACEDETECTOR_FACE) {
            found_frontal_face_detection = true;
            chosen_face_detection = face_detection;
            break;
        }
    }

    if (!found_frontal_face_detection) {
        chosen_face_detection = current_face_detections.back();
    }

    ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(
            chosen_face_detection.m_FaceRegion);
    bicv_HeadPoseParameters face_params =
            HpParams2RoiConverter::roi2hpparams(face_roi);

    static const float FACE_TRANSLATION_X_STDDEV = 10.0;
    static const float FACE_TRANSLATION_Y_STDDEV = 10.0;
    static const float FACE_SCALE_STDDEV = 0.1f;
    static const float FACE_EXCENTRICITY_STDDEV = 0.001f;

    static const float SAME_POSE_CORRECTION_COEFF = 1.0f;
    static const float OTHER_POSE_CORRECTION_COEFF = 0.5f;

    const unsigned pose = pose_index_from_face_orientation(
            chosen_face_detection.m_Pose);
    float coeff_pose = (state.m_ParamsCur.m_PoseIndex == pose) ?
            SAME_POSE_CORRECTION_COEFF :
            OTHER_POSE_CORRECTION_COEFF;

    // translation X
    boost::math::normal_distribution<float> normal_x_dist(
            face_params.m_TranslationX, FACE_TRANSLATION_X_STDDEV);
    // translation Y
    boost::math::normal_distribution<float> normal_y_dist(
            face_params.m_TranslationY, FACE_TRANSLATION_Y_STDDEV);
    // scale
    boost::math::normal_distribution<float> normal_s_dist(
            face_params.m_Scale, FACE_SCALE_STDDEV);
    // excentricity - approximately
    boost::math::normal_distribution<float> normal_e_dist(
            face_params.m_Excentricity, FACE_EXCENTRICITY_STDDEV);
    result *= pdf(normal_x_dist, state.m_ParamsCur.m_TranslationX)
            * pdf(normal_y_dist, state.m_ParamsCur.m_TranslationY)
            * pdf(normal_s_dist, state.m_ParamsCur.m_Scale)
            * pdf(normal_e_dist, state.m_ParamsCur.m_Excentricity)
            * coeff_pose;
    return result;
} // weight_correction_face

void bicv_ColourSegmentationTracker::derive_average_scale_distribution(
    const boost::circular_buffer<cvp_FaceDescriptor>& faces,
    float min_scale_stddev) {

    // cache mean scale of previous detections and set up prior on scale
    float scale;
    float mean_scale = 0.0f;
    float var_scale = 0.0f;
    BOOST_FOREACH(const cvp_FaceDescriptor& face_descriptor, faces) {
        CvRect face_rect = face_descriptor.m_FaceRegion;
        ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(face_rect);
        bicv_TrackerParameters face_params =
                HpParams2RoiConverter::roi2params(face_roi);
        scale = face_params.m_Scale;
        mean_scale += scale;
        var_scale += scale * scale;
    }

    mean_scale /= m_LastAssociatedFaceDetections.size();
    var_scale = sqrt(var_scale / m_LastAssociatedFaceDetections.size() -
            mean_scale * mean_scale);
    if (var_scale < min_scale_stddev) {
        var_scale = min_scale_stddev;
    }

//    cout << "Derived scale: "
//         << ", mean: " << mean_scale << ", stddev: " << var_scale << endl;

    m_ScaleAverageBasedPrior = boost::math::normal_distribution<float>(
            mean_scale, var_scale);

} // derive_average_scale_distribution

inline bicv_TrackerState bicv_ColourSegmentationTracker::correct_state(
        const bicv_TrackerState& state) {
    IplImage* image = m_DataProvider->image_buffer();
    const int width = image->width;
    const int height = image->height;


    ip_RoiWindow roi_state = HpParams2RoiConverter::params2roi(
            state.m_ParamsCur);

    assert(roi_state.m_iWidth > 0);
    assert(roi_state.m_iHeight > 0);

    if (roi_state.m_iFirstColumn + roi_state.m_iWidth > width) {
        roi_state.m_iFirstColumn -= (roi_state.m_iFirstColumn +
                roi_state.m_iWidth - width);
    }

    if (roi_state.m_iFirstRow + roi_state.m_iHeight > height) {
        roi_state.m_iFirstRow -= (roi_state.m_iFirstRow +
                roi_state.m_iHeight - height);
    }

    if (roi_state.m_iFirstColumn < 0) {
        roi_state.m_iFirstColumn = 0;
    }

    if (roi_state.m_iFirstRow < 0) {
        roi_state.m_iFirstRow = 0;
    }

    bicv_TrackerState corrected_state = state;
    corrected_state.m_ParamsCur = HpParams2RoiConverter::roi2params(roi_state);
    return corrected_state;
} // correct_state

bicv_TrackerState bicv_ColourSegmentationTracker::correct_state_by_skin_region(
        IplImage * ipImage, const bicv_TrackerState& state) {

    using namespace FaceColorModel;

    ip_RoiWindow roi_state = HpParams2RoiConverter::params2roi(
            state.m_ParamsCur);

    CvRect cvRoi = cvRect(roi_state.m_iFirstColumn, roi_state.m_iFirstRow,
            roi_state.m_iWidth, roi_state.m_iHeight);
    IplImage * temp_image = cvCreateImage(cvSize(roi_state.m_iWidth,
            roi_state.m_iHeight), ipImage->depth, ipImage->nChannels);
    try {
        cvSetImageROI(ipImage, cvRoi);
        cvCopy(ipImage, temp_image);
        cvResetImageROI(ipImage);
    } catch(...) {
        cvReleaseImage(&temp_image);
        return state;
    }

    // FCM_NUM_CHANNELS x M x N
    fcm_real * opLogLikelihood = new fcm_real [FCM_NUM_CHANNELS *
        roi_state.m_iWidth * roi_state.m_iHeight];

//    m_FaceColourModel->channel_log_likelihood(temp_image, opLogLikelihood);

    fcm_real mean_x = 0;
    fcm_real mean_y = 0;
    fcm_real mean_sq_x = 0;
    fcm_real mean_sq_y = 0;

    fcm_real sum_loglhoods = 0;
    fcm_real sum_loglhoods_temp = 0;
    fcm_real curlhood;

    fcm_real * loglhoods_ptr = opLogLikelihood;
    for (int row = 0; row < roi_state.m_iHeight; ++row) {
        sum_loglhoods_temp = 0;
        for (int col = 0; col < roi_state.m_iHeight; ++col) {
            curlhood = exp(*loglhoods_ptr++);
            sum_loglhoods_temp += curlhood;
            mean_x += curlhood * col;
            mean_sq_x += curlhood * col * col;
        }
        sum_loglhoods += sum_loglhoods_temp;
        mean_y += sum_loglhoods_temp * row;
        mean_sq_y += sum_loglhoods_temp * row * row;
    }

    mean_x /= sum_loglhoods; // * roi_state.m_iWidth;
    mean_y /= sum_loglhoods; // * roi_state.m_iHeight;
    mean_sq_x = sqrt(mean_sq_x / sum_loglhoods - mean_x * mean_x);
    mean_sq_y = sqrt(mean_sq_y / sum_loglhoods - mean_y * mean_y);

    mean_x /= roi_state.m_iWidth;
    mean_y /= roi_state.m_iHeight;
    mean_sq_x /= roi_state.m_iWidth;
    mean_sq_y /= roi_state.m_iHeight;

    fcm_real shift_x = (0.5 - mean_x) * roi_state.m_iWidth;
    fcm_real shift_y = (0.56 - mean_y) * roi_state.m_iHeight;
    //fcm_real scale_x = 0.14 / mean_sq_x;
    //fcm_real scale_y = 0.1953 / mean_sq_y;

    bicv_TrackerState corrected_state = state;
    corrected_state.m_ParamsCur.m_TranslationX += shift_x;
    corrected_state.m_ParamsCur.m_TranslationY += shift_y;
//    corrected_state.m_ParamsCur.m_Scale /= scale_x;
//    corrected_state.m_ParamsCur.m_Excentricity *= scale_y / scale_x;

    delete [] opLogLikelihood;
    cvReleaseImage(&temp_image);
    return corrected_state;

} // correct_state_by_skin_region

unsigned bicv_ColourSegmentationTracker::pose_index_from_face_orientation(
        OpenCvPlus::cvp_FaceDetectorPose face_pose) const {
    unsigned num_poses = m_FaceColourModel->pim_models_count();
    if (1 == num_poses) {
        return 0;
    }
    switch (face_pose) {
    case CVP_FACEDETECTOR_FACE:
        return 0;
    case CVP_FACEDETECTOR_PROFILE_LEFT:
        return 1;
    case CVP_FACEDETECTOR_PROFILE_RIGHT:
        return 2;
    default:
        return 0;
    }
} // pose_index_from_face_orientation


} // namespace BICV
