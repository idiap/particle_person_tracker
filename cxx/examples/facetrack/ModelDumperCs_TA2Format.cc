// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumperCs_TA2Format - class to dump main model data
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
#include "ModelDumperCs_TA2Format.h"                  // declaration of this

using namespace std;
using namespace BICV;
using namespace ImageProcessing;
using namespace OpenCvPlus;

//#define DUMP_PARTICLES
#define DUMP_TRACKER_MANAGER_DATA

/////////////////////////////// PUBLIC ///////////////////////////////////////

ModelDumperCs_TA2Format::ModelDumperCs_TA2Format(
        const std::string& filename,
        const MainModel_ColourSegmentationTracker* model) : m_Model(model),
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

} // ModelDumperCs_TA2Format

ModelDumperCs_TA2Format::~ModelDumperCs_TA2Format() {
    if (m_FileStream.good()) {
        m_FileStream  << endl << "</analysisresult>" << endl;
        m_FileStream.close();
    }
    m_Model = 0;
} // ~ModelDumperCs_TA2Format

void  ModelDumperCs_TA2Format::update() {

    if (!m_FileStream.good()) {
        return;
    }

    const int frame_rate_fps = 25;
    const float frame_delay_ms = 1000.0 / frame_rate_fps;

    boost::posix_time::time_duration td = boost::posix_time::milliseconds(
            frame_delay_ms * (m_ImageCount++));

//    boost::posix_time::ptime timestamp = m_Model->data_provider()->time();

    const list<bicv_ColourSegmentationTracker*>& trackers =
            m_Model->trackers();
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers) {
        dump_tracker(tracker, td);
    }

} // update

void  ModelDumperCs_TA2Format::dump_tracker(
        bicv_ColourSegmentationTracker * tracker,
        const boost::posix_time::time_duration& timestamp) {

    static const boost::posix_time::ptime epoch(
        boost::gregorian::date(1970,boost::gregorian::Jan,1));

    bicv_TrackerState dumped_state = mean(tracker->particle_distribution());
    ip_RoiWindow tracker_roi = HpParams2RoiConverter::params2roi(
            dumped_state.m_ParamsCur);

    m_FileStream << endl <<
        "<event type=\"face\" " <<  "id=\"" << tracker->id() << "\" " <<
        "time=\"" << (m_Model->data_provider()->time() - epoch).total_microseconds() << "\" " <<
        "frame=\"" << m_Model->data_provider()->image_id() << "\">" << endl <<
//        "<timestamp> " << timestamp.total_milliseconds() << " </timestamp>" << endl <<
        "<timestamp> " << m_Model->data_provider()->image_id() << " </timestamp>" << endl;

    const bicv_StateImageLikelihood * lhood = tracker->likelihood();
    IplImage * image = tracker->data_provider()->image();
#ifdef DUMP_PARTICLES
    m_FileStream << "<particles>" << endl;
    BOOST_FOREACH(const bicv_ColourSegmentationTracker::ParticleDistribution::element_type& weighted_particle,
            tracker->particle_distribution().elements()) {
        float lhood_val = lhood->evaluate(weighted_particle.first, image);
        m_FileStream << "<particle weight=\"" << weighted_particle.second << "\" "
                << "lhood=\"" << lhood_val << "\"> "
                << weighted_particle.first << " </particle>" << endl;
    }
    m_FileStream << "</particles>" << endl;
#endif

    m_FileStream << "<priors>" << endl <<
        "<scale mean=\"" << tracker->scale_prior().mean() << "\" stddev=\""
             << tracker->scale_prior().standard_deviation() << "\"/>"
             << endl << "</priors>" << endl;

    m_FileStream <<  "<current_faces>" << endl;
    list<OpenCvPlus::cvp_FaceDescriptor> faces =
            tracker->associated_face_detections_for_timestamp(
                    m_Model->data_provider()->time());
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
            m_Model->tracker_manager_observation(tracker);
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

    m_FileStream << "<value>" << endl <<
        "<TL> " << "<X> " << tracker_roi.m_iFirstColumn << " </X> " <<
                   "<Y> " << tracker_roi.m_iFirstRow << " </Y> " << " </TL>" << endl <<
        "<BR> " << "<X> " << tracker_roi.m_iFirstColumn + tracker_roi.m_iWidth - 1 << " </X> " <<
                   "<Y> " << tracker_roi.m_iFirstRow + tracker_roi.m_iHeight - 1 << " </Y> " << " </BR>" << endl <<
        // for the time being no confidence and smoothness
        // "<confidence> " << 1.0 << " </confidence> " << endl <<
        // "<smoothness> " << 0.0 << " </smoothness> " << endl <<
        "</value>" << endl;

    m_FileStream << "<scores>" << endl <<
        "<likelihood> " << lhood->evaluate(dumped_state, image) <<
        "</likelihood>" << endl <<
        "</scores>" << endl;
    m_FileStream <<
        "</event>" << endl;

} // dump_tracker
