// Copyright (c) 2011-2020 Idiap Research Institute
//
// FaceTrackerConfig - stores options for face tracker program
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/program_options.hpp>            // boost program options
#include <sstream>                              // string stream
#include <iostream>                             // string stream
#include <fstream>                              // STL input file stream


#include "opencvplus/cvp_FaceDetectorFactory.h"
#include "FaceTrackerConfig.hpp"                // declaration of this

using namespace std;
namespace po = boost::program_options;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const double DEFAULT_SCALE  = 1.0; // processed image width
static const unsigned DEFAULT_IMAGE_WIDTH  = 320; // processed image width
static const unsigned DEFAULT_IMAGE_HEIGHT = 240; // processed image height
static const unsigned DEFAULT_FPS = -1;           // default FPS rate
static const bool DEFAULT_ONLINE = false;         // default processing mode
static const unsigned DEFAULT_VISU = 0;           // disable GUI by default
static const long DEFAULT_FACE_DELAY_MS = 5000;   // face detector delay in ms
static const long DEFAULT_NUM_TRACKER_PARTICLES = 200;
static const string DEFAULT_FACE_DETECTOR_TYPE = "cv";

// dynamic model default values for priors
static const float DEFAULT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR = 0.45;
static const float DEFAULT_COEFF_TRACKER_AR_DYNMODEL_PRIOR = 0.2;
static const float DEFAULT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR = 0.45;
static const float DEFAULT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR = 0.8;

static const int DEFAULT_MIN_FACE_HEIGHT = 10;

// Face < 10% of image height are not tracked and deleted
static const double DEFAULT_MIN_FACE_HEIGHT_RATIO = 0.1;

static const string OPT_SCALE_STR  = "scale";     // Scale for images
static const string OPT_WIDTH_STR  = "width";     // image width option name
static const string OPT_HEIGHT_STR = "height";    // image height option name
static const string OPT_FPS_STR = "fps";          // source FPS option name
static const string OPT_HOG_MODEL_STR  = "hog_model";  // hog model option name
static const string OPT_SKIN_MODEL_STR = "skin_model"; // skin model option name

// face detector delay in milliseconds
static const string OPT_FACE_DELAY_STR = "face_detector_delay_ms";
// face detector type (opencv / ctu / torch3)
static const string OPT_FACE_DETECTOR_TYPE_STR = "face_detector_type";

// Face detector statistics options
static const string OPT_FD_FACE_STATS_STR = "fd_face_stats";
static const string OPT_FD_PROFILE_LEFT_STATS_STR = "fd_profile_left_stats";
static const string OPT_FD_PROFILE_RIGHT_STATS_STR = "fd_profile_right_stats";

static const string OPT_FD_ROI_OVERLAP_THRESHOLD = "fd_roi_overlap_threshold";
static const float  DEFAULT_ROI_OVERLAP = 0.95;

// background colour model
static const string OPT_BG_COL_MODEL_STR = "prior_color_model_background";
// clothes colour model
static const string OPT_CLOTHES_COL_MODEL_STR = "prior_color_model_clothes";
// hair colour model
static const string OPT_HAIR_COL_MODEL_STR = "prior_color_model_hair";
// skin colour model
static const string OPT_SKIN_COL_MODEL_STR = "prior_color_model_skin";
// PIM model
static const string OPT_PIM_MODEL_STR = "prior_pim";

// number of particles to use in head pose tracker
static const string OPT_NUM_TRACKER_PARTICLES_STR = "num_tracker_particles";

// motion-based dynamic model prior
static const string OPT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR_STR =
        "coeff_tracker_motion_dynmodel_prior";
// autoregressive dynamic model prior
static const string OPT_COEFF_TRACKER_AR_DYNMODEL_PRIOR_STR =
        "coeff_tracker_ar_dynmodel_prior";
// random search dynamic model prior
static const string OPT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR_STR =
        "coeff_tracker_rsearch_dynmodel_prior";
// face-based dynamic model prior
static const string OPT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR_STR =
        "coeff_tracker_face_dynmodel_prior";

static const string OPT_HEAD_POSE_SCOPE =
        "hp_scope"; // head pose rsb scope
static const string OPT_VFOA_SCOPE =
        "vfoa_scope"; // vfoa rsb scope
static const string OPT_USE_PROFILES_STR =
        "fd_use_profiles"; // use profiles detector ?

static const string OPT_STEREO_CALIBRATION_FILE_STR = "stereo_calibration_file";

// path to nod detector data
static const string OPT_NOD_DETECTOR_DATA_STR = "nod_detector_data";

// static const string OPT_NOD_DETECTOR_DATA_STR = "nod_detector_data";


FaceTrackerConfig::FaceTrackerConfig():
        m_EnableVideoOutputFile(false),
        m_Visu(DEFAULT_VISU),
        m_NumTrackerParticles(DEFAULT_NUM_TRACKER_PARTICLES),
        m_FaceDetectorType(OpenCvPlus::str2faceDetectorType(DEFAULT_FACE_DETECTOR_TYPE)),
        m_Scale(DEFAULT_SCALE),
        m_ImageWidth(DEFAULT_IMAGE_WIDTH),
        m_ImageHeight(DEFAULT_IMAGE_HEIGHT),
        m_Online(DEFAULT_ONLINE),
        m_MinFaceHeight(DEFAULT_MIN_FACE_HEIGHT),
        m_MinFaceHeightRatio(DEFAULT_MIN_FACE_HEIGHT_RATIO)
{
}


/* static */ void
FaceTrackerConfig::parse_command_line(int argc, char* argv[]) {

    po::options_description cmdline_options("General options");
    cmdline_options.add_options()
        ("help", "produce help message")
        ("config,c", po::value<string>(), "Configuration file")
        ("input,i",  po::value<string>(), "Input file")
        ("output,o", po::value<string>(), "Output file")
        ("dump,d",   po::value<string>(), "Dump file")
        ("online",   po::value<>(&m_Online)->zero_tokens(), "Online processing mode")
        ("visu,v",   po::value<unsigned>(), "Whether to show GUI")
        ("scale,s",  po::value<double>(), "Image rescale ratio")
        ("image_height,h",    po::value<unsigned>(), "Image height")
        ("image_width,w",     po::value<unsigned>(), "Image width")
        ("delay,t",           po::value<unsigned>(), "Detector delay (in ms)")
        ("particules,p",      po::value<unsigned>(), "Number of particules for the tracker")
        ("face_detector,f",   po::value<string>(), "Type of face detector")
        ("min_face_height,m", po::value<int>(), "Minimum face height to start tracking")
        ("min_face_height_ratio,M", po::value<double>(), "Minimum face height to continuetracking")
        ;

    po::store(po::parse_command_line(argc, argv, cmdline_options), m_VariablesMap);
    po::notify(m_VariablesMap);


    if (m_VariablesMap.count("help")) {
        ostringstream oss;
        oss << cmdline_options << endl;
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count("input")) {
        m_VideoInputDevice = m_VariablesMap["input"].as<string>();
    } else {
        ostringstream oss;
        oss << endl << "ERROR: Input file must be provided!" << endl << endl <<
                cmdline_options << endl;
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count("output")) {
        m_EnableVideoOutputFile = true;
        m_OutputVideoFile = m_VariablesMap["output"].as<string>();
    } else {
        m_EnableVideoOutputFile = false;
    }

    if (m_VariablesMap.count("dump")) {
        m_EnableDump = true;
        m_OutputDump = m_VariablesMap["dump"].as<string>();
    } else {
        m_EnableDump = false;
    }

    if (m_VariablesMap.count("config")) {
        parse_config_file(m_VariablesMap["config"].as<string>());
    }

    if (m_VariablesMap.count("image_width")) {
        m_ImageWidth = m_VariablesMap["image_width"].as<unsigned>();
        if(m_ImageWidth==0) m_ImageWidth = DEFAULT_IMAGE_WIDTH;
    }

    if (m_VariablesMap.count("image_height")) {
        m_ImageHeight = m_VariablesMap["image_height"].as<unsigned>();
        if(m_ImageHeight==0) m_ImageHeight = DEFAULT_IMAGE_HEIGHT;
    }

    if (m_VariablesMap.count("scale")) {
        m_Scale = m_VariablesMap["scale"].as<double>();
        if(m_Scale==0) m_Scale = DEFAULT_SCALE;
    }

    if (m_VariablesMap.count("face_detector")) {
      m_FaceDetectorType = OpenCvPlus::str2faceDetectorType(m_VariablesMap["face_detector"].as<string>());
    }

    if (m_VariablesMap.count("particules")) {
        m_NumTrackerParticles = m_VariablesMap["particules"].as<unsigned>();
    }

    if (m_VariablesMap.count("delay")) {
      m_FaceDetectorDelay = m_VariablesMap["delay"].as<unsigned>();
    }

    if (m_VariablesMap.count("visu")) {
      m_Visu = m_VariablesMap["visu"].as<unsigned>();
    }

    if (m_VariablesMap.count("min_face_height")) {
      m_MinFaceHeight = m_VariablesMap["min_face_height"].as<int>();
    }

    if (m_VariablesMap.count("min_face_height_ratio")) {
      m_MinFaceHeightRatio = m_VariablesMap["min_face_height_ratio"].as<double>();
    }

} // parse_command_line

/* static */ void
FaceTrackerConfig::parse_config_file(const std::string& fname)
{
    po::options_description cfgfile_options("Config options");
    cfgfile_options.add_options()
        (OPT_SCALE_STR.c_str(), po::value<double>()->default_value(DEFAULT_SCALE),
         "Image rescale ratio")
        (OPT_WIDTH_STR.c_str(), po::value<unsigned>()->default_value(DEFAULT_IMAGE_WIDTH),
         "Image width")
        (OPT_HEIGHT_STR.c_str(), po::value<unsigned>()->default_value(DEFAULT_IMAGE_HEIGHT),
         "Image height")
        (OPT_FPS_STR.c_str(), po::value<unsigned>()->default_value(DEFAULT_FPS),
         "Image provider FPS rate")
        (OPT_HOG_MODEL_STR.c_str(), po::value<string>(),
         "HoG model file path")
        (OPT_SKIN_MODEL_STR.c_str(), po::value<string>(),
         "Skin model file path")
        (OPT_FACE_DELAY_STR.c_str(), po::value<long>()->default_value(DEFAULT_FACE_DELAY_MS),
         "Face detector delay")
        (OPT_NUM_TRACKER_PARTICLES_STR.c_str(), po::value<unsigned>()->default_value(DEFAULT_NUM_TRACKER_PARTICLES),
         "Number of tracker particles")
        (OPT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR_STR.c_str(), po::value<float>()->default_value(DEFAULT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR),
         "Prior for tracker motion-based dynamic model")
        (OPT_COEFF_TRACKER_AR_DYNMODEL_PRIOR_STR.c_str(), po::value<float>()->default_value(DEFAULT_COEFF_TRACKER_AR_DYNMODEL_PRIOR),
         "Prior for tracker autoregressive dynamic model")
        (OPT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR_STR.c_str(), po::value<float>()->default_value(DEFAULT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR),
         "Prior for tracker random search-based dynamic model")
        (OPT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR_STR.c_str(), po::value<float>()->default_value(DEFAULT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR),
         "Prior for tracker face-based dynamic model")
        (OPT_BG_COL_MODEL_STR.c_str(), po::value<string>(),
         "Background colour model")
        (OPT_CLOTHES_COL_MODEL_STR.c_str(), po::value<string>(),
         "Clothes colour model")
        (OPT_HAIR_COL_MODEL_STR.c_str(), po::value<string>(),
         "Hair colour model")
        (OPT_SKIN_COL_MODEL_STR.c_str(), po::value<string>(),
         "Skin colour model")
        (OPT_PIM_MODEL_STR.c_str(), po::value<string>(),
         "PIM model")
        (OPT_FACE_DETECTOR_TYPE_STR.c_str(), po::value<string>(),
         "Face detector type (cv, openpose, openheadpose)")
        (OPT_USE_PROFILES_STR.c_str(), po::value<bool>(),
         "Use profile detector ( 0 = no, 1 = yes )")
        (OPT_FD_FACE_STATS_STR.c_str(), po::value<string>(),
         "Frontal face detector statistics file path")
        (OPT_FD_PROFILE_LEFT_STATS_STR.c_str(), po::value<string>(),
         "Left profile face detector statistics file path")
        (OPT_FD_PROFILE_RIGHT_STATS_STR.c_str(), po::value<string>(),
         "Right profile face detector statistics file path")
        (OPT_HEAD_POSE_SCOPE.c_str(), po::value<string>(),
         "Head Pose RSB scope")
        (OPT_VFOA_SCOPE.c_str(), po::value<string>(),
         "VFOA RSB scope")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OPENCV_FACE_CASCADE_STR.c_str(), po::value<string>(),
         "OpenCV face detector cascade path")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OPENCV_PROFILE_LEFT_CASCADE_STR.c_str(), po::value<string>(),
         "OpenCV profile left detector cascade path")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OPENCV_PROFILE_RIGHT_CASCADE_STR.c_str(), po::value<string>(),
         "OpenCV profile right detector cascade path")
        (OPT_STEREO_CALIBRATION_FILE_STR.c_str(), po::value<string>(),
         "Stereo camera pair calibration file")
        (OPT_NOD_DETECTOR_DATA_STR.c_str(), po::value<string>(),
         "Nod detector data file path")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OP_POSE_STR.c_str(), po::value<string>(),
         "OpenPose model type (COCO or BODY_25)")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OP_MODEL_STR.c_str(), po::value<string>(),
         "OpenPose model file")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OP_NET_RESOLUTION_STR.c_str(), po::value<string>(),
         "OpenPose net resolution")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OP_VISU_STR.c_str(), po::value<int>(),
         "Whether to diplay output of OpenPose")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_OHP_MODEL_STR.c_str(), po::value<string>(),
         "Path to OpenHeadPose folder")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_STR.c_str(), po::value<int>(),
         "Minimum height to start tracking")
        (OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_RATIO_STR.c_str(), po::value<double>(),
         "Minimum height ratio to continue tracking")
        (OPT_FD_ROI_OVERLAP_THRESHOLD.c_str(), po::value<float>()->default_value(DEFAULT_ROI_OVERLAP),
         "Overlap threshold between face detection and tracker")
        ;

    ifstream ifs(fname.c_str());
    if (!ifs) {
        ostringstream oss;
        oss << endl << "ERROR: Cannot open config file " << fname
            << "!" << endl;
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    po::store(po::parse_config_file(ifs, cfgfile_options), m_VariablesMap);
    po::notify(m_VariablesMap);

    if (m_VariablesMap.count(OPT_SCALE_STR)) {
        m_Scale = m_VariablesMap[OPT_SCALE_STR].as<double>();
    }

    if (m_VariablesMap.count(OPT_WIDTH_STR)) {
        m_ImageWidth = m_VariablesMap[OPT_WIDTH_STR].as<unsigned>();
    }

    if (m_VariablesMap.count(OPT_HEIGHT_STR)) {
        m_ImageHeight = m_VariablesMap[OPT_HEIGHT_STR].as<unsigned>();
    }

    if (m_VariablesMap.count(OPT_FPS_STR)) {
        m_Fps = m_VariablesMap[OPT_FPS_STR].as<unsigned>();
    }

    if (m_VariablesMap.count(OPT_FACE_DELAY_STR)) {
        m_FaceDetectorDelay = m_VariablesMap[OPT_FACE_DELAY_STR].as<long>();
    }

    if (m_VariablesMap.count(OPT_NUM_TRACKER_PARTICLES_STR)) {
        m_NumTrackerParticles = m_VariablesMap[OPT_NUM_TRACKER_PARTICLES_STR].
                as<unsigned>();
    }

    if (m_VariablesMap.count(OPT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR_STR)) {
        m_DynamicModelConfig.m_MBWeight =
                m_VariablesMap[OPT_COEFF_TRACKER_MOTION_DYNMODEL_PRIOR_STR].
                as<float>();
    }

    if (m_VariablesMap.count(OPT_COEFF_TRACKER_AR_DYNMODEL_PRIOR_STR)) {
        m_DynamicModelConfig.m_ARWeight =
                m_VariablesMap[OPT_COEFF_TRACKER_AR_DYNMODEL_PRIOR_STR].
                as<float>();
    }

    if (m_VariablesMap.count(OPT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR_STR)) {
        m_DynamicModelConfig.m_RSWeight =
                m_VariablesMap[OPT_COEFF_TRACKER_RSEARCH_DYNMODEL_PRIOR_STR].
                as<float>();
    }

    if (m_VariablesMap.count(OPT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR_STR)) {
        m_DynamicModelConfig.m_FBWeight =
                m_VariablesMap[OPT_COEFF_TRACKER_FACE_DYNMODEL_PRIOR_STR].
                as<float>();
    }

    if (m_VariablesMap.count(OPT_HOG_MODEL_STR)) {
        m_TrainedHogModelPath = m_VariablesMap[OPT_HOG_MODEL_STR].as<string>();
    }

    if (m_VariablesMap.count(OPT_SKIN_MODEL_STR)) {
        m_TrainedSkinModelPath = m_VariablesMap[OPT_SKIN_MODEL_STR].as<string>();
    }

    if (m_VariablesMap.count(OPT_BG_COL_MODEL_STR)) {
        m_FaceColorModelConfig.m_BackgroundColourModel =
                m_VariablesMap[OPT_BG_COL_MODEL_STR].as<string>();
    } else {
        ostringstream oss;
        oss << "Background colour model file not provided in config!";
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count(OPT_CLOTHES_COL_MODEL_STR)) {
        m_FaceColorModelConfig.m_ClothesColourModel =
                m_VariablesMap[OPT_CLOTHES_COL_MODEL_STR].as<string>();
    } else {
        ostringstream oss;
        oss << "Clothes colour model file not provided in config!";
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count(OPT_HAIR_COL_MODEL_STR)) {
        m_FaceColorModelConfig.m_HairColourModel =
                m_VariablesMap[OPT_HAIR_COL_MODEL_STR].as<string>();
    } else {
        ostringstream oss;
        oss << "Hair colour model file not provided in config!";
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count(OPT_SKIN_COL_MODEL_STR)) {
        m_FaceColorModelConfig.m_SkinColourModel =
                m_VariablesMap[OPT_SKIN_COL_MODEL_STR].as<string>();
    } else {
        ostringstream oss;
        oss << "Skin colour model file not provided in config!";
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count(OPT_PIM_MODEL_STR)) {
        m_FaceColorModelConfig.m_PimModel =
                m_VariablesMap[OPT_PIM_MODEL_STR].as<string>();
    } else {
        ostringstream oss;
        oss << "PIM model file not provided in config!";
        throw FaceTracker_CmdLine_Parser_Exception(oss.str());
    }

    if (m_VariablesMap.count(OPT_HEAD_POSE_SCOPE)) {
        m_HpScope =
                m_VariablesMap[OPT_HEAD_POSE_SCOPE].as<string>();
    }

    if (m_VariablesMap.count(OPT_VFOA_SCOPE)) {
        m_VfoaScope =
                m_VariablesMap[OPT_VFOA_SCOPE].as<string>();
    }

    if (m_VariablesMap.count(OPT_USE_PROFILES_STR)) {
        m_UseProfiles =
                m_VariablesMap[OPT_USE_PROFILES_STR].as<bool>();
    }
    if (m_VariablesMap.count(OPT_FD_FACE_STATS_STR)) {
        m_FrontalFaceStatsFilePath =
                m_VariablesMap[OPT_FD_FACE_STATS_STR].as<string>();
    }
    if (m_VariablesMap.count(OPT_FD_PROFILE_LEFT_STATS_STR)) {
        m_ProfileLeftFaceStatsFilePath =
                m_VariablesMap[OPT_FD_PROFILE_LEFT_STATS_STR].as<string>();
    }
    if (m_VariablesMap.count(OPT_FD_PROFILE_RIGHT_STATS_STR)) {
        m_ProfileRightFaceStatsFilePath =
                m_VariablesMap[OPT_FD_PROFILE_RIGHT_STATS_STR].as<string>();
    }

    if (m_VariablesMap.count(OPT_FACE_DETECTOR_TYPE_STR)) {
      m_FaceDetectorType = OpenCvPlus::str2faceDetectorType(m_VariablesMap[OPT_FACE_DETECTOR_TYPE_STR].as<string>());
    }

    if (m_VariablesMap.count(OPT_STEREO_CALIBRATION_FILE_STR)) {
        m_StereoCalibrationFile =
                m_VariablesMap[OPT_STEREO_CALIBRATION_FILE_STR].as<string>();
    }
    if (m_VariablesMap.count(OPT_NOD_DETECTOR_DATA_STR)) {
        m_NodDetectorFilePath =
                m_VariablesMap[OPT_NOD_DETECTOR_DATA_STR].as<string>();
        m_NodDetectorFilePathProvided = true;
    } else {
        m_NodDetectorFilePathProvided = false;
    }
    if (m_VariablesMap.count(OPT_FD_ROI_OVERLAP_THRESHOLD)) {
        m_RoiOverlapThreshold =
          m_VariablesMap[OPT_FD_ROI_OVERLAP_THRESHOLD].as<float>();
    }
    if (m_VariablesMap.count(OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_STR)) {
        m_MinFaceHeight =
          m_VariablesMap[OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_STR].as<int>();
    }
    if (m_VariablesMap.count(OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_RATIO_STR)) {
        m_MinFaceHeightRatio =
          m_VariablesMap[OpenCvPlus::cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_RATIO_STR].as<double>();
    }


} // parse_config_file


FaceTracker_CmdLine_Parser_Exception::
FaceTracker_CmdLine_Parser_Exception(const std::string& msg) :
    std::runtime_error(msg) {
} // FaceTracker_CmdLine_Parser_Exception
