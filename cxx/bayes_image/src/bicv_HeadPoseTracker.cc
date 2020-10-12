// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseTracker - class to track head pose
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                       // foreach loop
#include <boost/random.hpp>                        // taus88 RNG
#include <boost/math/distributions/normal.hpp>     // normal distribution
#include <ctime>                                   // time
#include <list>                                    // STL list
#include <sstream>                                 // STL string stream
#include <numeric>                                 // accumulate

// PROJECT INCLUDES
#ifdef __STEREO_MATCHER_FOUND__
#include <image_processing/ip_DisparityImageProcessor.h>   // disparity/depth
#endif

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTracker.h>                   // declaration of this
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // ROI converter
#include <bayes_image/bicv_HeadPoseLikelihoodModel.h>           // Likelihood

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;

// Dumps tracker particles
//#define ENABLE_TRACKER_DUMP

// Updates face colour model when face is detected
#define UPDATE_FACE_COLOUR_MODEL
//#define DUMP_FACE_COLOUR_MODEL

#ifdef DUMP_FACE_COLOUR_MODEL
#include <boost/filesystem.hpp>                    // for create_directories
#endif

// KLUDGE: remove this directive, add one for both ip_SkinColourProcessor.cc and
// this file!!!
#define USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE

#ifdef ENABLE_TRACKER_DUMP
static int tracker_counter = 1;
int GLOBAL_FRAME_COUNTER = 1;
#endif

//////////////////////////// LOCAL CONSTANTS /////////////////////////////////

// Gaussian distribution prior parameters for HEAD POSE component
static const float PAN_PRIOR_MEAN = 0;
static const float PAN_PRIOR_STDDEV = 45;
static const float TILT_PRIOR_MEAN = 0;
static const float TILT_PRIOR_STDDEV = 30;
static const float ROLL_PRIOR_MEAN = 0;
static const float ROLL_PRIOR_STDDEV = 30;
// Gaussian distribution prior parameters for SCALE component
static const float SCALE_PRIOR_MEAN = 1.0;
static const float SCALE_PRIOR_STDDEV = 0.5;
static const float SCALE_PRIOR_MIN_STDDEV = 0.1;
static const float SCALE_WEIGHT_CORRECTION_MIN = 0.1;
// Gaussian distribution prior parameters for EXCENTRICITY component
static const float EXCENTRICITY_PRIOR_MEAN = 1;
//static const float EXCENTRICITY_PRIOR_STDDEV = 0.05;
static const float EXCENTRICITY_PRIOR_STDDEV = 0.1;

// Face observation standard deviations
static const float FACE_OBSERVATION_TRANSLATION_X_STDDEV = 20.0;
static const float FACE_OBSERVATION_TRANSLATION_Y_STDDEV = 20.0;
static const float FACE_OBSERVATION_SCALE_STDDEV = 0.1f;

static const int LIKELIHOOD_VALUES_COUNT = 1;
static const float LIKELIHOOD_TRACKING_THRESHOLD = 1e-15;

static const unsigned NUM_FACE_DETECTIONS_TO_KEEP = 10;

// Time interval for which to keep the history of motion estimations
static const double MOTION_ESTIMATION_TIME_INTERVAL_MICROSEC = 3e6;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static int GLOBAL_TRACKER_COUNTER = 0;

/////////////////////////////// PUBLIC ///////////////////////////////////////

namespace BICV {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

bicv_HeadPoseTracker::bicv_HeadPoseTracker(
    bicv_HeadPoseMixtureDynamicModel * dyn_model,
    const Likelihood * lhood,
    ip_ImageProvider * data_provider,
    ip_ImageProvider * grayscale_provider,
    ip_GradientImageProcessor * gradient_provider,
    ip_ColourSkinFeatureProducer * skin_colour_producer,
    ip_Dense2DMotionProcessorInria * motion_processor,
    FaceColorModel::FaceColorModel * face_colour_model,
    ip_ImageProvider * depth_provider,
    bicv_HeadPoseSkinModel * trained_skin_model,
    const ip_HogFeatureParameters& hog_params,
    const ip_SkinFeatureParameters& skin_params,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain,
    const cvp_FaceDetectorStatisticsStorage * fd_stats_storage,
    unsigned num_particles) :

    m_ParticleDistribution(bicv_HeadPoseARTrackerState(), num_particles),
    m_ParticleDistributionRng(generate_random_seed()),
    m_DynamicModel(dyn_model),
    m_DynamicModelRng(generate_random_seed()),
    m_Likelihood(lhood),
    m_ScaleAverageBasedPrior(SCALE_PRIOR_MEAN, SCALE_PRIOR_STDDEV),
    m_LastAssociatedFaceDetections(NUM_FACE_DETECTIONS_TO_KEEP),
    m_DataProvider(data_provider),
    m_GrayscaleProvider(grayscale_provider),
    m_GradientProvider(gradient_provider),
    m_MotionProcessor(motion_processor),
    mFaceColourModel(face_colour_model),
    m_DepthProvider(depth_provider),
    mSkinColourProducer(skin_colour_producer),
    m_TrainedSkinModel(trained_skin_model),
    m_HeadPoseDomain(head_pose_domain),
    m_FaceDetectorStatisticsStorage(fd_stats_storage),
    m_LhoodValues(LIKELIHOOD_VALUES_COUNT) {

    m_Id = GLOBAL_TRACKER_COUNTER++;
    m_FaceIdx = -1;
//    cout << "Tracker, initialized face colour model with " << mFaceColourModel << endl;

    m_IntegralHogProvider = new ip_IntegralGradientHistogramProcessor(
            m_GradientProvider->getGradientAngleProvider(),
            m_GradientProvider->getGradientMagnitudeProvider(),
            hog_params.mNumHistBins);
    m_SkinColourProvider = new ip_SkinColourProcessor(
            m_DataProvider, mFaceColourModel, skin_params.mSkinMaskValue);
    m_IntegralSkinProvider = new ip_IntegralSkinMaskProcessor(
            m_SkinColourProvider);

    m_HogFeatureProducer = new ip_HogFeatureProducerRounded(
        m_IntegralHogProvider, hog_params);
    m_SkinFeatureProducer = new ip_SkinFeatureProducer(
        m_IntegralSkinProvider, skin_params);

    // initialize motion estimator parameters
    m_MotionModel = new CMotion2DModel();
    m_MotionEstimator = new CMotion2DEstimator();
    ip_Dense2DMotionProcessorInria::apply_config_options(
            m_MotionProcessor->getConfig(), m_MotionModel, m_MotionEstimator);

#ifdef ENABLE_TRACKER_DUMP
    ostringstream oss;
    oss << "tracker_" << (tracker_counter++) << "_dump.txt";
    m_Dumpfile.open(oss.str().c_str());
    m_IterationCount = 1;
#endif

} // bicv_HeadPoseTracker

/* static */
bicv_HeadPoseTracker::~bicv_HeadPoseTracker() {
#ifdef ENABLE_TRACKER_DUMP
    m_Dumpfile.close();
#endif

    // delete motion estimator parameters
    delete m_MotionEstimator;
    delete m_MotionModel;

    delete m_SkinFeatureProducer;
    delete m_HogFeatureProducer;
    delete m_IntegralSkinProvider;
    delete m_SkinColourProvider;
    delete m_IntegralHogProvider;
    delete mFaceColourModel;
    delete m_DynamicModel;
} // ~bicv_HeadPoseTracker

void bicv_HeadPoseTracker::init(const bicv_HeadPoseARTrackerState& state) {
    vector<ParticleDistribution::element_type>& elements =
        m_ParticleDistribution.elements();
    BOOST_FOREACH(ParticleDistribution::element_type& element, elements) {
        element.first = state;
        element.second = 1.0f;
    }
} // init

void bicv_HeadPoseTracker::init(
    const OpenCvPlus::cvp_FaceDescriptor& init_candidate,
    const boost::posix_time::ptime & timestamp) {

    // initialize sampling distribution
    boost::normal_distribution<float> gaussian_dist;
    boost::variate_generator<boost::mt19937&,
        boost::normal_distribution<float> > gaussian(m_DynamicModelRng,
        gaussian_dist);

    // get parameters for statistics of head pose given face detection
    const cvp_FaceDetectorStatistics& fd_stats =
            m_FaceDetectorStatisticsStorage->face_detector_statistics(
                    init_candidate.m_Pose);

    // convert face descriptor to tracker state
    bicv_HeadPoseParameters face_params =
            HpParams2RoiConverter::roi2hpparams(
                    ip_RoiWindow::from_CvRect(init_candidate.m_FaceRegion));

    // resample all particles
    vector<ParticleDistribution::element_type>& elements =
        m_ParticleDistribution.elements();

    HeadPose hp;
    bicv_HeadPoseParameters hp_params;
    BOOST_FOREACH(ParticleDistribution::element_type& element, elements) {
        hp_params.m_HeadPose.pan(fd_stats.m_MeanHeadPose.val[0] + gaussian() *
               fd_stats.m_StddevHeadPose.val[0]);
        hp_params.m_HeadPose.tilt(fd_stats.m_MeanHeadPose.val[1] + gaussian() *
               fd_stats.m_StddevHeadPose.val[1]);
        hp_params.m_HeadPose.roll(fd_stats.m_MeanHeadPose.val[2] + gaussian() *
               fd_stats.m_StddevHeadPose.val[2]);
        hp_params.m_TranslationX = face_params.m_TranslationX +
              fd_stats.m_MeanBBoxTransform.val[0] + gaussian() *
              fd_stats.m_StddevBBoxTransform.val[0];
        hp_params.m_TranslationY = face_params.m_TranslationY +
                fd_stats.m_MeanBBoxTransform.val[1] + gaussian() *
                fd_stats.m_StddevBBoxTransform.val[1];
        hp_params.m_Scale = face_params.m_Scale *
                fd_stats.m_MeanBBoxTransform.val[2] + gaussian() *
                face_params.m_Scale * fd_stats.m_StddevBBoxTransform.val[2];
        hp_params.m_Excentricity = face_params.m_Excentricity *
                fd_stats.m_MeanBBoxTransform.val[3] /
                fd_stats.m_MeanBBoxTransform.val[2] + gaussian() *
                face_params.m_Excentricity *
                fd_stats.m_StddevBBoxTransform.val[3];
        hp_params.m_Time = timestamp;

        element.first.m_HeadPoseParamsCur = hp_params;
        element.first.m_HeadPoseParamsPrev = hp_params;
        element.second = 1.0f;
    }

    // save detected face into associated faces
    observe_face(init_candidate);

} // init

void bicv_HeadPoseTracker::init_with_unknown_head_pose(
        const bicv_HeadPoseARTrackerState& state) {
    vector<ParticleDistribution::element_type>& elements =
        m_ParticleDistribution.elements();

    boost::normal_distribution<float> gaussian_dist;
    boost::variate_generator<boost::mt19937&,
        boost::normal_distribution<float> > gaussian(m_DynamicModelRng,
        gaussian_dist);

    BOOST_FOREACH(ParticleDistribution::element_type& element, elements) {
        HeadPose hp(PAN_PRIOR_MEAN + gaussian() * PAN_PRIOR_STDDEV,
            TILT_PRIOR_MEAN + gaussian() * TILT_PRIOR_STDDEV,
            ROLL_PRIOR_MEAN + gaussian() * ROLL_PRIOR_STDDEV);
        element.first = state;
        element.first.m_HeadPoseParamsCur.m_HeadPose = hp;
        element.first.m_HeadPoseParamsPrev.m_HeadPose = hp;
        element.second = 1.0f;
    }
} // init_with_unknown_head_pose

void bicv_HeadPoseTracker::set_skin_colour_model(
        const cvp_SkinColourModel& skin_colour_model) {
#ifndef USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE
    m_SkinColourProvider->reset_colour_model();
    m_SkinColourProvider->update_colour_model(skin_colour_model);
#endif
} // set_skin_colour_model

void bicv_HeadPoseTracker::set_skin_colour_model(
        const ImageProcessing::ip_RoiWindow& roi) {
#ifndef USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE
    int shink_rate = 3;
    ip_RoiWindow roi_shrinked;
     roi_shrinked.m_iFirstColumn = roi.m_iFirstColumn +
             round(static_cast<float>(roi.m_iWidth) / shink_rate);
     roi_shrinked.m_iFirstRow = roi.m_iFirstRow +
             round(static_cast<float>(roi.m_iHeight) / shink_rate);
     roi_shrinked.m_iWidth = roi.m_iWidth -
             round(2 * static_cast<float>(roi.m_iWidth) / shink_rate);
     roi_shrinked.m_iHeight = roi.m_iHeight -
             round(2 * static_cast<float>(roi.m_iHeight) / shink_rate);
     m_SkinColourProvider->set_threshold(1e8);
     m_SkinColourProvider->invalidate();
     IplImage * mask_img = m_SkinColourProvider->image(roi_shrinked);
     m_SkinColourProvider->update_colour_model(m_DataProvider->image(),
             mask_img, roi_shrinked, 0.8);

//     m_SkinColourProvider->set_threshold(8);
//     m_SkinColourProvider->invalidate();
//     mask_img = m_SkinColourProvider->image(roi_shrinked);
//     m_SkinColourProvider->update_colour_model(m_DataProvider->image(),
//             mask_img, roi_shrinked);
     m_SkinColourProvider->invalidate();
     m_SkinColourProvider->image(roi_shrinked);
#endif
} // set_skin_colour_model

void bicv_HeadPoseTracker::update_skin_colour_model(
        const cvp_SkinColourModel& skin_colour_model) {
#ifndef USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE
m_SkinColourProvider->update_colour_model(skin_colour_model);
#endif
} // update_skin_colour_model

void bicv_HeadPoseTracker::update_skin_colour() {

#ifndef USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE
    const ip_SkinTemplate& head_template =
            m_TrainedSkinModel->head_mean_template();

    ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
            mean(m_ParticleDistribution).m_HeadPoseParamsCur);

    const ip_ColourSkinTemplate& skin_colour_template =
            mSkinColourProducer->compute_feature(roi);

    ip_SkinTemplate red_colour_template(
            skin_colour_template.mRedColourTemplate);
    ip_SkinTemplate green_colour_template(
            skin_colour_template.mGreenColourTemplate);
    red_colour_template /= skin_colour_template.mRedColourTemplate +
            skin_colour_template.mGreenColourTemplate +
            skin_colour_template.mBlueColourTemplate;
    green_colour_template /= skin_colour_template.mRedColourTemplate +
            skin_colour_template.mGreenColourTemplate +
            skin_colour_template.mBlueColourTemplate;

    ip_SkinColourModel previous_model = m_SkinColourProvider->get_colour_model();
    ip_SkinTemplate label_template(head_template);
    for (unsigned j = 0; j < label_template.size(); ++j) {
        label_template[j] = (distance(previous_model, green_colour_template[j],
                red_colour_template[j]) < 3) ? 1 : 0;
    }

    ip_SkinColourModel model;

    for (int i = 1; i < 10; ++i) {
        if (label_template.sum() <= 0) {
            break;
        }
        model.mRedMean = ((red_colour_template * head_template * label_template).sum() /
                (head_template * label_template).sum());
        model.mRedVar = (red_colour_template * red_colour_template *
                head_template * label_template).sum() /
                (head_template * label_template).sum() -
                model.mRedMean * model.mRedMean;
        model.mGreenMean = ((green_colour_template * head_template * label_template).sum() /
                (head_template * label_template).sum());
        model.mGreenVar = (green_colour_template * green_colour_template *
                head_template * label_template).sum() /
                (head_template * label_template).sum() -
                model.mGreenMean * model.mGreenMean;
        model.mGreenRedCovar = ((green_colour_template * red_colour_template *
                head_template * label_template).sum() /
                (head_template * label_template).sum() -
                model.mGreenMean * model.mRedMean);
        for (unsigned j = 0; j < label_template.size(); ++j) {
            label_template[j] = (distance(model, green_colour_template[j],
                    red_colour_template[j]) < 3) ? 1 : 0;
        }
//        cout << endl;
//        for (unsigned j = 0; j < label_template.size(); ++j) {
//            cout << label_template[j] << " ";
//        }
//        cout << endl;
//        for (unsigned j = 0; j < label_template.size(); ++j) {
//            cout << distance_template[j] << " ";
//        }
//        cout << endl;
    }

    if (label_template.sum() > 0) {
        m_SkinColourProvider->update_colour_model(model, 1e-2);
    }
#endif
} // update_skin_colour

void bicv_HeadPoseTracker::iterate() {

    invalidate_providers();

    vector<ParticleDistribution::element_type> & elements =
        m_ParticleDistribution.elements();

    // get input image width and height and define image ROI
    const int width = m_DataProvider->image_buffer()->width;
    const int height = m_DataProvider->image_buffer()->height;
    const ip_RoiWindow global_roi(0, 0, width, height);

    // get current and previous timestamps
    const boost::posix_time::ptime current_data_time = m_DataProvider->time();
    const boost::posix_time::ptime previous_data_time =
            elements[0].first.m_HeadPoseParamsCur.m_Time;
    // estimate motion
    ip_MotionParameters motion_estim(CMotion2DModel::MDL_NMAX_COEF);

    list<cvp_FaceDescriptor> associated_face_detections =
            associated_face_detections_for_timestamp(current_data_time);

    bicv_HeadPoseARTrackerState mean_state = mean(particle_distribution());
    ip_RoiWindow mean_roi = HpParams2RoiConverter::hpparams2roi(
            mean_state.m_HeadPoseParamsCur);
    static const float support_scale = 1.2;
    ip_RoiWindow mean_roi_scaled = scale(mean_roi, support_scale);
    ip_RoiWindow motion_estimation_roi;

    static const boost::posix_time::ptime epoch(
            boost::gregorian::date(1970,boost::gregorian::Jan,1));

    if (intersect(global_roi, mean_roi_scaled, motion_estimation_roi)) {
        m_MotionProcessor->reestimate4roi(motion_estimation_roi,
                                          m_MotionModel,
                                          m_MotionEstimator);
        motion_estim.m_BoundingBox = motion_estimation_roi;
        m_MotionModel->getParameters(&motion_estim.m_Parameters[0]);
        motion_estim.m_Flags[0] = true;
        motion_estim.m_Flags[1] = true;
        motion_estim.m_Flags[2] = true;
        double ts_microsec = (current_data_time - epoch).total_microseconds();
        // verify strict ordering of timestamps
        if (m_MotionEstimList.empty() ||
            (m_MotionEstimList.back().first < ts_microsec)) {
            m_MotionEstimList.push_back(make_pair(ts_microsec, motion_estim));
            revise_motion_estimation_list();
        }
    } else {
        motion_estim.m_Flags[0] = false;
        motion_estim.m_Flags[1] = false;
        motion_estim.m_Flags[2] = false;
    }

    // prepare dynamic model for sampling
    m_DynamicModel->prepare(current_data_time,
                            previous_data_time,
                            motion_estim,
                            associated_face_detections);

    // cout << "Sampling: " << endl;
    typedef pair<bicv_HeadPoseARTrackerState,
                 bicv_HeadPoseARTrackerState> ParticlePair;

    const double todeg = 180.0/M_PI;
    cvp_FaceDescriptor fd;
    {
      if(!associated_face_detections.empty())
        {
          // cvp_FaceDescriptor chosen_face_detection;
          BOOST_FOREACH(const cvp_FaceDescriptor& face_detection,
                        associated_face_detections) {
            if (face_detection.m_Pose == CVP_FACEDETECTOR_FACE) {
              fd = face_detection;
              break;
            }

          }
          // std::cout << current_data_time << " : "
          //           << fd.m_HeadPose << std::endl;

        }
    }

    // predict particles using the dynamic model
    list<ParticlePair> new_particles;
    for (unsigned i = 0; i < elements.size(); ++i)
      {
        bicv_HeadPoseARTrackerState particle =
            sample(m_ParticleDistributionRng, m_ParticleDistribution);

        // if(!associated_face_detections.empty())
        //   {
        //     std::cout << " - " << particle.m_HeadPoseParamsCur.m_HeadPose << std::endl;
        //     // particle.m_HeadPoseParamsCur.m_HeadPose.pan(fd.m_HeadPose.pan()*todeg);
        //     // particle.m_HeadPoseParamsCur.m_HeadPose.tilt(fd.m_HeadPose.tilt()*todeg);
        //     std::cout << " + " << particle.m_HeadPoseParamsCur.m_HeadPose << std::endl;
        //   }

        bicv_HeadPoseARTrackerState new_particle =
                m_DynamicModel->sample(m_DynamicModelRng, particle);

        if(!associated_face_detections.empty())
          {
            // std::cout << " < " << new_particle.m_HeadPoseParamsCur.m_HeadPose << std::endl;
            if(fd.m_HeadPose.pan()!=0 && fd.m_HeadPose.tilt() !=0)
              {
                new_particle.m_HeadPoseParamsCur.m_HeadPose.pan(fd.m_HeadPose.pan()*todeg);
                new_particle.m_HeadPoseParamsCur.m_HeadPose.tilt(fd.m_HeadPose.tilt()*todeg);
              }
            // std::cout << " < " << new_particle.m_HeadPoseParamsPrev.m_HeadPose << std::endl;
            // std::cout << " > " << new_particle.m_HeadPoseParamsCur.m_HeadPose << std::endl;
          }


        correct_particle(new_particle, width, height);

        new_particles.push_back(make_pair(particle, new_particle));

        // std::cout << "----" << std::endl;
        // cout << particle << endl;
        // cout << new_particle << endl;
      }

#ifdef ENABLE_TRACKER_DUMP
    m_Dumpfile << "frame " << GLOBAL_FRAME_COUNTER << endl;
#endif

    // compute the bounding box for all the particles
    int first_col = numeric_limits<int>::max();
    int last_col = numeric_limits<int>::min();
    int first_row = numeric_limits<int>::max();
    int last_row = numeric_limits<int>::min();
    BOOST_FOREACH(const ParticlePair& particle_pair, new_particles) {
        ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
                particle_pair.second.m_HeadPoseParamsCur);
        if (first_col > roi.m_iFirstColumn) {
            first_col = roi.m_iFirstColumn;
        }
        if (last_col < roi.m_iFirstColumn + roi.m_iWidth) {
            last_col = roi.m_iFirstColumn + roi.m_iWidth;
        }
        if (first_row > roi.m_iFirstRow) {
            first_row = roi.m_iFirstRow;
        }
        if (last_row < roi.m_iFirstRow + roi.m_iHeight) {
            last_row = roi.m_iFirstRow + roi.m_iHeight;
        }
    }
    ip_RoiWindow particles_roi(first_col,
                               first_row,
                               last_col - first_col,
                               last_row - first_row);
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
            mFaceColourModel->adapt_to(source_img, chosen_face_detection.m_FaceRegion);
#ifdef DUMP_FACE_COLOUR_MODEL
            const unsigned num_palettes =
                    mFaceColourModel->current_palette_bundle()->size();
            vector<string> palette_fnames;
            palette_fnames.reserve(num_palettes);
            const string prefix = "/scratch/vkhalidov/results_temp/dump_fcm/";
            ostringstream oss;
            // verify that the dump folder exists
            oss << prefix << id();
            boost::filesystem::create_directories(oss.str());
            // dump palettes
            for (unsigned i = 0; i < num_palettes; ++i) {
                oss.str("");
                oss << prefix << id() << '/' << "palette_" << i << '_'
                    << m_DataProvider->image_id() << ".dat";
                palette_fnames.push_back(oss.str());
            }
            mFaceColourModel->current_palette_bundle()->save_to_files(palette_fnames);
            // dump pims
            for (unsigned i = 0; i < num_palettes; ++i) {
                oss.str("");
                oss << prefix << id() << '/' << "pim_" << i << '_'
                    << m_DataProvider->image_id() << ".dat";
            }
            mFaceColourModel->current_pim()->save_to_file(oss.str());
            // dump image
            CvRect roi = chosen_face_detection.m_FaceRegion;
            if ((source_img->width != chosen_face_detection.m_ImageWidth) ||
                (source_img->height != chosen_face_detection.m_ImageHeight)) {
                roi.x = chosen_face_detection.m_FaceRegion.x * source_img->width / chosen_face_detection.m_ImageWidth;
                roi.y = chosen_face_detection.m_FaceRegion.y * source_img->height / chosen_face_detection.m_ImageHeight;
                roi.width = chosen_face_detection.m_FaceRegion.width * source_img->width / chosen_face_detection.m_ImageWidth;
                roi.height = chosen_face_detection.m_FaceRegion.height * source_img->height / chosen_face_detection.m_ImageHeight;
            }
            cvSetImageROI(source_img, roi);
            IplImage * face_img = cvCreateImage(cvSize(roi.width, roi.height), source_img->depth, source_img->nChannels);
            cvCopy(source_img, face_img);
            oss.str("");
            oss << prefix << id() << '/' << "face_"
                << m_DataProvider->image_id() << ".png";
            cvSaveImage(oss.str().c_str(), face_img);
            cvReleaseImage(&face_img);
            cvResetImageROI(source_img);
#endif
        }
    }
#endif

    // request images for the ROI from hog and skin providers
    m_IntegralHogProvider->image(particles_roi_corrected);
    m_IntegralSkinProvider->image(particles_roi_corrected);

    // reweight particles by observations
    const list<ParticlePair>::const_iterator first_ii = new_particles.begin();
    const list<ParticlePair>::const_iterator last_ii = new_particles.end();
    list<ParticlePair>::const_iterator ii = first_ii;
    list<float> weights;

    float lhood_val;
    float dynamics_val;
    float proposal_val;
    float correction;

    for(; ii != last_ii; ++ii) {
        ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
                ii->second.m_HeadPoseParamsCur);
//        cout << ii->m_HeadPoseParamsCur << endl;
        bicv_SkinHogObservation skinhog_obs(
                m_HogFeatureProducer->compute_feature(roi),
                m_SkinFeatureProducer->compute_feature(roi));
//        cout << "likelihood " << m_Likelihood->evaluate(*ii, skinhog_obs) *
//                weight_correction(*ii) << endl;
        lhood_val = m_Likelihood->evaluate(ii->second, skinhog_obs);
        dynamics_val = m_DynamicModel->evaluate_dynamics(ii->first, ii->second);
        proposal_val = m_DynamicModel->evaluate_proposal(ii->first, ii->second);
        correction = weight_correction(ii->second);
        weights.push_back(lhood_val * dynamics_val / proposal_val * correction);

#ifdef ENABLE_TRACKER_DUMP
        m_Dumpfile << "particle " << (*ii) << " " <<
                dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(m_Likelihood)->
                        evaluate_HoG(*ii, skinhog_obs) << " " <<
                dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(m_Likelihood)->
                        evaluate_Skin(*ii, skinhog_obs) << " " <<
                weight_correction_prior(*ii) << " " <<
                weight_correction_face(*ii) << " " <<
                m_Likelihood->evaluate(*ii, skinhog_obs) * weight_correction(*ii) <<
                endl;
#endif
    }

    // write results to the particle distribution
    list<float>::const_iterator jj = weights.begin();

    vector<ParticleDistribution::element_type>::iterator kk = elements.begin();
    for (ii = first_ii; ii != last_ii; ++ii, ++jj, ++kk) {
        kk->first = ii->second;
        kk->second = *jj;
    }

    m_LhoodValues.push_back(internal_tracking_score());

} // iterate

CvScalar bicv_HeadPoseTracker::scene_coordinates_3D() const {
    // define mean tracker region of interest
    bicv_HeadPoseARTrackerState mean_state = mean(particle_distribution());
    ip_RoiWindow mean_roi = HpParams2RoiConverter::hpparams2roi(
            mean_state.m_HeadPoseParamsCur);

    CvScalar result = cvScalarAll(0);

#ifdef __STEREO_MATCHER_FOUND__
    ip_DisparityImageProcessor * disparity_processor =
            dynamic_cast<ip_DisparityImageProcessor*>(m_DepthProvider);
    if (disparity_processor) {
        disparity_processor->image();
        if (disparity_processor->has_valid_disparity()) {
            // convert mean tracker ROI to match depth image size
            IplImage * disparity_image =
                    disparity_processor->disparity_image();
            IplImage * tracker_image = m_DataProvider->image();
            if ((disparity_image->width == tracker_image->width) &&
                (disparity_image->height == tracker_image->height)) {
                result = disparity_processor->scene_coordinates_3D_disparity(
                        mean_roi);
                return result;
            } else {
                float coeff_x = static_cast<float>(disparity_image->width) /
                    tracker_image->width;
                float coeff_y = static_cast<float>(disparity_image->height) /
                    tracker_image->height;
                ip_RoiWindow scaled_mean_roi(
                    static_cast<int>(mean_roi.m_iFirstColumn * coeff_x),
                    static_cast<int>(mean_roi.m_iFirstRow * coeff_y),
                    static_cast<int>(mean_roi.m_iWidth * coeff_x),
                    static_cast<int>(mean_roi.m_iHeight * coeff_y));
                result = disparity_processor->scene_coordinates_3D_disparity(
                        scaled_mean_roi);
                return result;
            }
        }
    }
#endif

    // if no depth provider is defined or depth provider didn't work correctly
    // use dummy calibration parameters and tracker scale to estimate depth
    IplImage * tracker_image = m_DataProvider->image();
    static const float CALIBRATION_CONST =
        static_cast<float>(tracker_image->width + tracker_image->height) / 2;
    static const float AVERAGE_HEAD_HEIGHT_MM = 235;
    result.val[2] = CALIBRATION_CONST * AVERAGE_HEAD_HEIGHT_MM /
            mean_roi.m_iHeight;
    result.val[0] = (mean_roi.m_iFirstColumn +
            static_cast<float>(mean_roi.m_iWidth - tracker_image->width) / 2)
            * result.val[2] / CALIBRATION_CONST;
    result.val[1] = (mean_roi.m_iFirstRow +
            static_cast<float>(mean_roi.m_iHeight - tracker_image->height) / 2)
            * result.val[2] / CALIBRATION_CONST;
    return result;
} // scene_coordinates_3D

bool bicv_HeadPoseTracker::should_continue_tracking() const {
    if (!m_LhoodValues.empty()) {
        float mean_lhood = accumulate(
                m_LhoodValues.begin(), m_LhoodValues.end(), 0.0) /
                m_LhoodValues.size();
        return (mean_lhood > LIKELIHOOD_TRACKING_THRESHOLD);
    } else {
        return true;
    }
}

float bicv_HeadPoseTracker::internal_tracking_score() const {
    bicv_HeadPoseARTrackerState mean_state = mean(m_ParticleDistribution);
    ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
            mean_state.m_HeadPoseParamsCur);
    bicv_SkinHogObservation skinhog_obs(
            m_HogFeatureProducer->compute_feature(roi),
            m_SkinFeatureProducer->compute_feature(roi));

    bicv_HeadPoseLikelihoodModel::value_type hog_lhood =
            dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(m_Likelihood)->
                evaluate_HoG(mean_state, skinhog_obs);
    bicv_HeadPoseLikelihoodModel::value_type skin_lhood =
            dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(m_Likelihood)->
                evaluate_Skin(mean_state, skinhog_obs);

    //float prior_pose = weight_correction_prior(mean_state);
    //float prior_face = weight_correction_face(mean_state);

    return (hog_lhood * skin_lhood);
} // internal_tracking_score

void
bicv_HeadPoseTracker::
observe_face(const cvp_FaceDescriptor& face_descriptor)
{
  CvRect face_region = face_descriptor.m_FaceRegion;
  if((face_region.x > 0)     &&
     (face_region.y > 0)     &&
     (face_region.width > 0) &&
     (face_region.height > 0))
    {
      m_LastAssociatedFaceDetections.push_back(face_descriptor);
    }

  derive_average_scale_distribution(m_LastAssociatedFaceDetections,
                                    SCALE_PRIOR_MIN_STDDEV);

} // observe_face

const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor> &
bicv_HeadPoseTracker::associated_face_detections() const {
    return m_LastAssociatedFaceDetections;
} // associated_face_detections

std::list<OpenCvPlus::cvp_FaceDescriptor>
bicv_HeadPoseTracker::associated_face_detections_for_timestamp(
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

float bicv_HeadPoseTracker::mode_likelihood() const {
     bicv_HeadPoseARTrackerState mode_state = mode(m_ParticleDistribution);
    ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
        mode_state.m_HeadPoseParamsCur);
    bicv_SkinHogObservation skinhog_obs(
        m_HogFeatureProducer->compute_feature(roi),
        m_SkinFeatureProducer->compute_feature(roi));
    return m_Likelihood->evaluate(mode_state, skinhog_obs) *
            weight_correction(mode_state);
} // mode_likelihood

/////////////////////////////// PRIVATE //////////////////////////////////////

void bicv_HeadPoseTracker::invalidate_providers() {
    m_IntegralHogProvider->invalidate();
    m_SkinColourProvider->invalidate();
    m_IntegralSkinProvider->invalidate();
} // invalidate_providers

inline
unsigned bicv_HeadPoseTracker::generate_random_seed() const {
    static const unsigned NUM_BOOTSTRAP_ITER = 100;
    boost::taus88 rng(static_cast<unsigned>(time(0)));
    unsigned result;
    for (unsigned i = 0; i < NUM_BOOTSTRAP_ITER; ++i) {
        rng();
    }
    return result = rng();
} // generate_random_seed

inline float bicv_HeadPoseTracker::weight_correction(
        const bicv_HeadPoseARTrackerState& state) const {

    float weight_correction_prior_part = weight_correction_prior(state);
    float weight_correction_face_part = 1.0f;
//    float weight_correction_face_part = weight_correction_face(state);
    return weight_correction_prior_part * weight_correction_face_part;

}

inline float bicv_HeadPoseTracker::weight_correction_prior(
        const bicv_HeadPoseARTrackerState& state) const {

    HeadPose hp(state.m_HeadPoseParamsCur.m_HeadPose);

    using namespace boost::math;

//    normal_distribution<float> normal_pan_dist(
//            PAN_PRIOR_MEAN, PAN_PRIOR_STDDEV);
//    normal_distribution<float> normal_tilt_dist(
//            TILT_PRIOR_MEAN, TILT_PRIOR_STDDEV);

//    float result = pdf(normal_pan_dist, hp.pan())
//         * pdf(normal_tilt_dist, hp.tilt());
    float result = 1.0f;

    boost::normal_distribution<float> normal_roll_dist(
            ROLL_PRIOR_MEAN, ROLL_PRIOR_STDDEV);
    boost::normal_distribution<float> normal_excentricity_dist(
            EXCENTRICITY_PRIOR_MEAN, EXCENTRICITY_PRIOR_STDDEV);

    float correction_term =
            pdf(m_ScaleAverageBasedPrior, state.m_HeadPoseParamsCur.m_Scale);

    result *= max(correction_term, SCALE_WEIGHT_CORRECTION_MIN);
    return result;
}

inline float bicv_HeadPoseTracker::weight_correction_face(
        const bicv_HeadPoseARTrackerState& state) const {

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

    const cvp_FaceDetectorStatistics& fd_stats =
            m_FaceDetectorStatisticsStorage->face_detector_statistics(
                    chosen_face_detection.m_Pose);
    ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(
            chosen_face_detection.m_FaceRegion);
    bicv_HeadPoseParameters face_params =
            HpParams2RoiConverter::roi2hpparams(face_roi);

    // translation X
    boost::math::normal_distribution<float> normal_x_dist(
            face_params.m_TranslationX + fd_stats.m_MeanBBoxTransform.val[0],
            fd_stats.m_StddevBBoxTransform.val[0]);
    // translation Y
    boost::math::normal_distribution<float> normal_y_dist(
            face_params.m_TranslationY + fd_stats.m_MeanBBoxTransform.val[1],
            fd_stats.m_StddevBBoxTransform.val[1]);
    // scale
    boost::math::normal_distribution<float> normal_s_dist(
            face_params.m_Scale * fd_stats.m_MeanBBoxTransform.val[2],
            face_params.m_Scale * fd_stats.m_StddevBBoxTransform.val[2]);
    // excentricity - approximately
    boost::math::normal_distribution<float> normal_e_dist(
            face_params.m_Excentricity * fd_stats.m_MeanBBoxTransform.val[3] /
            fd_stats.m_MeanBBoxTransform.val[2],
            EXCENTRICITY_PRIOR_STDDEV
//            face_params.m_Excentricity * fd_stats.m_StddevBBoxTransform.val[3]
            );
    // pan
    boost::math::normal_distribution<float> normal_pan_dist(
            fd_stats.m_MeanHeadPose.val[0],
            fd_stats.m_StddevBBoxTransform.val[0]);
    // tilt
    boost::math::normal_distribution<float> normal_tilt_dist(
            fd_stats.m_MeanHeadPose.val[1],
            fd_stats.m_StddevBBoxTransform.val[1]);
    // roll
    boost::math::normal_distribution<float> normal_roll_dist(
            fd_stats.m_MeanHeadPose.val[2],
            fd_stats.m_StddevBBoxTransform.val[2]);
    result *= pdf(normal_x_dist, state.m_HeadPoseParamsCur.m_TranslationX)
            * pdf(normal_y_dist, state.m_HeadPoseParamsCur.m_TranslationY)
            * pdf(normal_s_dist, state.m_HeadPoseParamsCur.m_Scale)
            * pdf(normal_e_dist, state.m_HeadPoseParamsCur.m_Excentricity)
            * pdf(normal_pan_dist, state.m_HeadPoseParamsCur.m_HeadPose.pan())
            * pdf(normal_tilt_dist, state.m_HeadPoseParamsCur.m_HeadPose.tilt());
    return result;
}

void
bicv_HeadPoseTracker::
derive_average_scale_distribution(const boost::
                                  circular_buffer<OpenCvPlus::
                                  cvp_FaceDescriptor>& faces,
                                  float min_scale_stddev)
{
  // cache mean scale of previous detections and set up prior on scale
  float scale;
  float mean_scale = 0.0f;
  float var_scale = 0.0f;

  BOOST_FOREACH(const cvp_FaceDescriptor& face_descriptor, faces) {
    CvRect face_rect = face_descriptor.m_FaceRegion;
    ip_RoiWindow face_roi = ip_RoiWindow::from_CvRect(face_rect);
    bicv_HeadPoseParameters face_params =
      HpParams2RoiConverter::roi2hpparams(face_roi);
    const cvp_FaceDetectorStatistics& fd_stats =
      m_FaceDetectorStatisticsStorage->
      face_detector_statistics(face_descriptor.m_Pose);

    scale = face_params.m_Scale * fd_stats.m_MeanBBoxTransform.val[2];
    mean_scale += scale;
    var_scale += scale * scale;
    //        mean_scale_stddev += face_params.m_Scale *
    //                fd_stats.m_StddevBBoxTransform.val[2];
  }

  mean_scale /= m_LastAssociatedFaceDetections.size();
  var_scale /= m_LastAssociatedFaceDetections.size();
  // May only happen because of precision problems:
  if (var_scale < mean_scale * mean_scale) {
    var_scale = min_scale_stddev;
  } else {
    var_scale = sqrt(var_scale - mean_scale * mean_scale);
    if (var_scale != var_scale || var_scale < min_scale_stddev) {
      var_scale = min_scale_stddev;
    }
  }

  m_ScaleAverageBasedPrior =
    boost::math::normal_distribution<float>(mean_scale, var_scale);

}

inline void bicv_HeadPoseTracker::correct_particle(
        bicv_HeadPoseARTrackerState& particle, int width, int height) {
    HeadPose head_pose = particle.m_HeadPoseParamsCur.m_HeadPose;
    ip_RoiWindow roi = HpParams2RoiConverter::hpparams2roi(
            particle.m_HeadPoseParamsCur);
    if (roi.m_iFirstColumn < 0) {
        roi.m_iFirstColumn = 0;
    }
    if (roi.m_iFirstRow < 0) {
        roi.m_iFirstRow = 0;
    }
    if (roi.m_iFirstColumn + roi.m_iWidth > width) {
        roi.m_iFirstColumn = width - roi.m_iWidth;
    }
    if (roi.m_iFirstRow + roi.m_iHeight > height) {
        roi.m_iFirstRow = height - roi.m_iHeight;
    }
    particle.m_HeadPoseParamsCur = HpParams2RoiConverter::roi2hpparams(roi);
    particle.m_HeadPoseParamsCur.m_HeadPose = head_pose;
} // correct_particle

void bicv_HeadPoseTracker::revise_motion_estimation_list() {
    const double last_ts = m_MotionEstimList.back().first;
    list<TimedMotionEstim>::iterator it_beg = m_MotionEstimList.begin();
    while (it_beg->first + MOTION_ESTIMATION_TIME_INTERVAL_MICROSEC <
        last_ts) {
        m_MotionEstimList.pop_front();
        it_beg = m_MotionEstimList.begin();
    }
} // revise_motion_estimation_list

} // namespace BICV
