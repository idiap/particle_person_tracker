// Copyright (c) 2011-2020 Idiap Research Institute
//
// FaceTrackerConfig - stores options for face tracker program
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __FACETRACKERCONFIG_HPP__
#define __FACETRACKERCONFIG_HPP__

// SYSTEM INCLUDES
#include <string>                               // standard string
#include <stdexcept>                            // for runtime_error
#include <boost/program_options.hpp>            // boost program options

// PROJECT INCLUDES
#include <opencvplus/cvp_FaceDetectorFactory.h> // for face detector options
#include <opencvplus/FaceColorModel.h>          // for face color model config
#include <bayes_image/bicv_HeadPoseMixtureDynamicModelConfig.h>

/// @brief Class to store options for face tracker program
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    22.06.2011

class FaceTrackerConfig {

public:

    /// Constructor, initializes the option descriptions
    FaceTrackerConfig();

    /// Parses command line arguments and stores the options
    /// in the corresponding fields
    /// @param argc Arguments count
    /// @param argv Arguments values
    void parse_command_line(int argc, char* argv[]);

    /// variables map that keeps all pairs of option names and values
    boost::program_options::variables_map m_VariablesMap;

    /// Video input device name, can be a file, a camera etc.
    std::string m_VideoInputDevice;

    /// Flag indicating whether video output into file should be enabled
    bool        m_EnableVideoOutputFile;

    /// Video output file name, ignored if m_EnableVideoOutputFile flag
    /// is not set
    std::string m_OutputVideoFile;

    /// Flag indicating whether video output should be shown
    unsigned    m_Visu;

    /// Flag indicating whether dump should be enabled
    bool        m_EnableDump;

    /// Dump output location (e.g. file, RSB), ignored if m_EnableDump flag is not set
    std::string m_OutputDump;

    /// Path to the trained HOG model
    std::string m_TrainedHogModelPath;

    /// Path to the trained skin model
    std::string m_TrainedSkinModelPath;

    /// Face detector delay
    long m_FaceDetectorDelay;

    /// Number of particles for head pose tracker
    unsigned m_NumTrackerParticles;

    /// Dynamic models configuration containing prior weights
    BICV::bicv_HeadPoseMixtureDynamicModelConfig m_DynamicModelConfig;

    /// Face detector type (OpenCV, CTU, Torch, RSB)
    int m_FaceDetectorType;

    /// face colour model configuration data containing paths to model files
    FaceColorModel::FaceColorModelConfig m_FaceColorModelConfig;

    /// Rescale ration before processing image
    double m_Scale;
    /// Input image width
    unsigned m_ImageWidth;
    /// Input image height
    unsigned m_ImageHeight;
    /// Input FPS rate
    unsigned m_Fps;

    // process data online / offline
    bool m_Online;

    ///RSB scope
    std::string m_HpScope;
    std::string m_VfoaScope;
    std::string m_LeftCamScope;
    std::string m_RightCamScope;

    std::string m_StereoCalibrationFile;

    //use profiles ?
    bool m_UseProfiles;

    // paths to face detector statistics files
    std::string m_FrontalFaceStatsFilePath;
    std::string m_ProfileLeftFaceStatsFilePath;
    std::string m_ProfileRightFaceStatsFilePath;
    // path to nod detector trained data
    std::string m_NodDetectorFilePath;
    bool m_NodDetectorFilePathProvided;

    float m_RoiOverlapThreshold;

    /// Minimum face height to start tracking (Only Open[Head]Pose for now)
    int m_MinFaceHeight;
    double m_MinFaceHeightRatio;

private:

    /// Parses config file and stores the options in the corresponding fields
    /// @param fname Config file name
    void parse_config_file(const std::string& fname);

};

/// @brief Class to report command line parser errors
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    22.06.2011

class FaceTracker_CmdLine_Parser_Exception : public std::runtime_error {

public:
    FaceTracker_CmdLine_Parser_Exception(const std::string& msg = "");

};

#endif /* FACETRACKERCONFIG_HPP_ */
