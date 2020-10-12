// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumper_TA2Format - class to dump main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                         // foreach loop
#include "boost/date_time/posix_time/posix_time.hpp" // boost posix time

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>  // parameters2roi converter
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>  // parameters2roi converter

// LOCAL INCLUDES
#include "ModelDumper_TA2Format.h"                  // declaration of this

using namespace std;
using namespace BICV;
using namespace VFOA;
using namespace ImageProcessing;
using namespace OpenCvPlus;

//#define DUMP_PARTICLES
#define DUMP_TRACKER_MANAGER_DATA

/////////////////////////////// PUBLIC ///////////////////////////////////////

ModelDumper_TA2Format::ModelDumper_TA2Format(
        const std::string& filename, const MainModel* model) :
        ModelDumper(filename, model),
        m_ImageCount(0) {

    m_FileStream.open(filename.c_str());

    if (!m_FileStream.good()) {
        cout << "Warning! Model dump to file " << filename <<
                " could not be started!" << endl;
    } else {
        m_FileStream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl <<
            "<!DOCTYPE analysisresult SYSTEM \"analysisresult_v0_1.dtd\">" << endl <<
            "<analysisresult>" << endl <<
            "<skip_frames>0</skip_frames>" << endl << endl;
    }

} // ModelDumper_TA2Format

/* virtual */ ModelDumper_TA2Format::~ModelDumper_TA2Format() {
    if (m_FileStream.good()) {
        m_FileStream  << endl << "</analysisresult>" << endl;
        m_FileStream.close();
    }
} // ~ModelDumper_TA2Format

/* virtual */ void  ModelDumper_TA2Format::update() {

    if (!m_FileStream.good()) {
        return;
    }

    const int frame_rate_fps = 25;
    const float frame_delay_ms = 1000.0 / frame_rate_fps;

    boost::posix_time::time_duration td = boost::posix_time::milliseconds
      (static_cast<long>(frame_delay_ms * (m_ImageCount++)));

//    boost::posix_time::ptime timestamp = m_Model->data_provider()->time();

    const list<bicv_HeadPoseTracker*>& trackers = model()->head_pose_trackers();
    BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers) {
        dump_tracker(tracker, td);
    }

} // update

void  ModelDumper_TA2Format::dump_tracker(bicv_HeadPoseTracker * tracker,
        const boost::posix_time::time_duration& timestamp) {

    static const boost::posix_time::ptime epoch(
            boost::gregorian::date(1970,boost::gregorian::Jan,1));

    bicv_HeadPoseARTrackerState dumped_state = mean(
            tracker->particle_distribution());
    ip_RoiWindow main_roi = HpParams2RoiConverter::hpparams2roi(
            dumped_state.m_HeadPoseParamsCur);
    cvp_HeadPoseDiscreteDomain::HeadPose main_hp =
        dumped_state.m_HeadPoseParamsCur.m_HeadPose;

    ip_HogFeatureProducerRounded * hog_feature_producer =
            tracker->hog_feature_producer();
    ip_SkinFeatureProducer * skin_feature_producer =
            tracker->skin_feature_producer();
    const bicv_HeadPoseTracker::Likelihood * likelihood = tracker->likelihood();
    const bicv_HeadPoseLikelihoodModel * likelihood_hp =
            dynamic_cast<const bicv_HeadPoseLikelihoodModel*>(tracker->likelihood());

    const MainModel * main_model = model();

//    VfoaDistribution vfoa_distr = main_model->vfoa_model()->
//            compute_vfoa_distribution_Hellinger(tracker);
//    vfoa_CognitiveVfoaModelObjectInfo obj_info = mode(vfoa_distr);


    m_FileStream << endl <<
        "<event type=\"face\" id=\"" << tracker->id() << "\" " <<
        "time=\"" << (main_model->data_provider()->time() - epoch).total_microseconds() << "\" " <<
        "frame=\"" << main_model->data_provider()->image_id() << "\">" << endl <<
//        "<timestamp> " << timestamp.total_milliseconds() << " </timestamp>" << endl <<
        "<timestamp> " << main_model->data_provider()->image_id() << " </timestamp>" << endl <<
        "<value>" << endl;

#ifdef DUMP_PARTICLES
    m_FileStream << "<particles>" << endl;
    BOOST_FOREACH(const bicv_HeadPoseTracker::ParticleDistribution::element_type& weighted_particle,
            tracker->particle_distribution().elements()) {
        const ip_HistogramTemplate& hog_feature =
                hog_feature_producer->compute_feature(main_roi);
        const ip_SkinTemplate& skin_feature =
                skin_feature_producer->compute_feature(main_roi);
        bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);
        float hog_lhood = likelihood_hp->evaluate_HoG(weighted_particle.first, skinhog_obs);
        float skin_lhood = likelihood_hp->evaluate_Skin(weighted_particle.first, skinhog_obs);

        m_FileStream << "<particle weight=\"" << weighted_particle.second << "\" "
                << "hog=\"" << hog_lhood << "\" " << "skin=\"" << skin_lhood << "\"> "
                << weighted_particle.first << " </particle>" << endl;
    }
    m_FileStream << "</particles>" << endl;
#endif

    const ip_HistogramTemplate& hog_feature =
            hog_feature_producer->compute_feature(main_roi);
    const ip_SkinTemplate& skin_feature =
            skin_feature_producer->compute_feature(main_roi);
    bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);

    m_FileStream << "<priors>" << endl <<
        "<scale mean=\"" << tracker->scale_prior().mean() << "\" stddev=\"" << tracker->scale_prior().standard_deviation() << "\"/>" << endl <<
        "</priors>" << endl <<
        "<current_faces>" << endl;
    list<OpenCvPlus::cvp_FaceDescriptor> faces =
            tracker->associated_face_detections_for_timestamp(
                    main_model->data_provider()->time());
    BOOST_FOREACH(const OpenCvPlus::cvp_FaceDescriptor& face, faces) {
        m_FileStream << "<face>" << face << "</face>" << endl;
    }
    m_FileStream << "</current_faces>" << endl;
    m_FileStream << "<all_faces>" << endl;
    const boost::circular_buffer<OpenCvPlus::cvp_FaceDescriptor> & all_faces =
        tracker->associated_face_detections();
    BOOST_FOREACH(const OpenCvPlus::cvp_FaceDescriptor& face, all_faces) {
        m_FileStream << "<face>" << face << "</face>" << endl;
    }
    m_FileStream << "</all_faces>" << endl;

#ifdef DUMP_TRACKER_MANAGER_DATA
    const BayesImage::bi_ObjectObservation * tracker_obs =
            main_model->tracker_manager_observation(tracker);
    if (tracker_obs) {
        m_FileStream << "<manager>" << endl;
        m_FileStream << "<detection_flag>"
                     << tracker_obs->m_bDetectionAssociated
                     << "</detection_flag>" << endl;
        m_FileStream << "<scores "
                     << "lhood_mean=\"" << tracker_obs->m_fLikelihoodMean <<  "\" "
                     << "var_norm_x=\"" << tracker_obs->m_fCurVarianceNorm[0] <<  "\" "
                     << "var_norm_y=\"" << tracker_obs->m_fCurVarianceNorm[1] <<  "\" "
                     << "track_mem=\"" << tracker_obs->m_fTrackingMemoryValue <<  "\" "
                     << "lhood_drop=\"" << tracker_obs->m_fHTMmax_Likelihood -
                         tracker_obs->m_fHTM_Likelihood << "\" "
                     << "var_drop=\"" << tracker_obs->m_fHTMmax_Variance -
                         tracker_obs->m_fHTM_Variance <<  "\" "
                     << "var_inc=\"" << tracker_obs->m_fHTU_Variance -
                     tracker_obs->m_fHTUmin_Variance << "\" "
                     << "/>" << endl;
        m_FileStream << "</manager>" << endl;
    }
#endif

    m_FileStream <<
        "<TL> " << "<X> " << main_roi.m_iFirstColumn << " </X> " <<
                   "<Y> " << main_roi.m_iFirstRow << " </Y> " << " </TL>" << endl <<
        "<BR> " << "<X> " << main_roi.m_iFirstColumn + main_roi.m_iWidth - 1 << " </X> " <<
                   "<Y> " << main_roi.m_iFirstRow + main_roi.m_iHeight - 1 << " </Y> " << " </BR>" << endl <<
        "<HP> " << "<pan> " << main_hp.pan() << " </pan> " <<
                   "<tilt> " << main_hp.tilt() << " </tilt> " <<
                   "<roll> " << main_hp.roll() << " </roll> " << " </HP>" << endl <<
//        "<VFOA> " << obj_info.mName << " </VFOA>" << endl <<
        // for the time being no confidence and smoothness
        // "<confidence> " << 1.0 << " </confidence> " << endl <<
        // "<smoothness> " << 0.0 << " </smoothness> " << endl <<
        "</value>" << endl <<
        "<scores>" << endl <<
        "<likelihood> " << likelihood->evaluate(dumped_state, skinhog_obs) << " </likelihood>" << endl;
    if (likelihood_hp) {
        m_FileStream <<
            "<hog> " << likelihood_hp->evaluate_HoG(dumped_state, skinhog_obs) << " </hog>" << endl <<
            "<skin> " << likelihood_hp->evaluate_Skin(dumped_state, skinhog_obs) << " </skin>" << endl;
    }
    m_FileStream << "</scores>" << endl;

    m_FileStream << "</event>" << endl;

} // dump_tracker
