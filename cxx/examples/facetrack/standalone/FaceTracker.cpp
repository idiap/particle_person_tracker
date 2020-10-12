/**
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief application to track faces, estimate head poses and the
 * associated visual foci of attention
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#ifdef __ROSINTEGRATION_FOUND__
#include <ros/ros.h>
#endif

// PROJECT INCLUDES
#include <image_processing/ip_Exceptions.h>              // exceptions
#include <image_processing/ip_ImageProvider.h>           // image provider
#ifdef __RSBINTEGRATION_FOUND__
#include <image_processing/ip_RsbImageProvider.h>        // RSB image provider
#endif
#include <image_processing/ip_DataImageProvider.h>       // OpenCV image provider
#include <image_processing/ip_FaceDetectorExecutable.h>  // executable for face detector
#include <image_processing/ip_ImageProviderFactory.h>    // image provider factory
#include <utils/ut_Timer.h>                              // FD timer

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                // config for face tracker
#include "MainModel.h"                          // main model
#include "MainWindow.h"                         // main window
#include "ModelDumperFactory.h"                 // model dumper factory

// SYSTEM INCLUDES
#include <boost/thread/thread.hpp>              // boost threading
//#include "pantheios_parameters.h"

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace TTrackUtils;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const string MAIN_WINDOW_NAME = "Face Tracker Main Window";

/////////////////////////////// PUBLIC ///////////////////////////////////////

#define USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
#define __RSB_OUTPUT_ENABLED__
//#define ENABLE_TRACKER_DUMP
#ifdef ENABLE_TRACKER_DUMP
extern int GLOBAL_FRAME_COUNTER;
#endif

int main (int argc, char **argv) {

    cvNamedWindow(MAIN_WINDOW_NAME.c_str(), CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
    cvResizeWindow(MAIN_WINDOW_NAME.c_str(), 640, 480);
    cvMoveWindow(MAIN_WINDOW_NAME.c_str(), 0, 0);

    // Parse input arguments and config
    FaceTrackerConfig config;

    // Parse input arguments and config
    try {
        config.parse_command_line(argc, argv);
    } catch(FaceTracker_CmdLine_Parser_Exception& e) {
        cerr << e.what() << endl;
        return 1;
    }
    #ifdef __ROSINTEGRATION_FOUND__
        ros::init(argc, argv, "facetrack");
    #endif

    // Initialize data provider from the given device
    ImageProcessing::ip_ImageProvider * data_provider;
    try {
        data_provider = ip_ImageProviderFactory::instance()->
                create_image_provider(
                        config.m_VideoInputDevice,
                        config.m_ImageWidth, config.m_ImageHeight,
                        config.m_Fps, !(config.m_Online));
    } catch(ImageProcessing::ip_Exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    // For RSB stereo provider specify scopes
    #ifdef __RSBINTEGRATION_FOUND__
    if ((data_provider->id() == IP_IMG_PROVIDER_STEREO_RSB) ||
        (data_provider->id() == IP_IMG_PROVIDER_RSB)) {
        ip_RsbImageProvider * rsb_provider = dynamic_cast<ip_RsbImageProvider*>(
                data_provider);
        rsb_provider->set_stereo_scope_names(config.m_LeftCamScope,
                config.m_RightCamScope);
    }
    #endif


    cout << "Image provider: " << data_provider
         << ", source " << config.m_VideoInputDevice
         << ", type: "  << str(data_provider->id())
         << ", image size " << data_provider->image_buffer()->width
         << " x " << data_provider->image_buffer()->height << endl << flush;

    // Prepare face detector types based on config
    std::list<cvp_FaceDetectorType> fdTypes;
    switch(config.m_FaceDetectorType){
        case 1: fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_FACE); break;
        case 2: fdTypes.push_back(CVP_FACEDETECTOR_CTU); break;
        // case 3: fdTypes.push_back(CVP_FACEDETECTOR_TORCH); break;
        case 4: fdTypes.push_back(CVP_FACEDETECTOR_RSB_FACE); break;
        case 5: fdTypes.push_back(CVP_FACEDETECTOR_OPENPOSE); break;
        default: fdTypes.push_back(CVP_FACEDETECTOR_UNKNOWN);
    }
    if (config.m_UseProfiles){
        switch(config.m_FaceDetectorType){
            case 1:
            case 2:
            case 3:
                fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_PROFILE_LEFT);
                fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_PROFILE_RIGHT);
                break;
            case 4:
                fdTypes.push_back(CVP_FACEDETECTOR_RSB_PROFILE_LEFT);
                fdTypes.push_back(CVP_FACEDETECTOR_RSB_PROFILE_RIGHT);
                break;
        }
    }
    // Construct face detector group from the provided types
    cvp_FaceDetectorGroup * fd_group =
      new cvp_FaceDetectorGroup(fdTypes, config.m_VariablesMap);
    IplImage * example_image = data_provider->image_buffer();
    fd_group->prepare(example_image, 1.0);

    // Initialize main model using the data provider and config options
    MainModel * main_model = new MainModel(data_provider, fd_group, config);

#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
    ip_FaceDetectorExecutable * face_detector_exec =
            new ip_FaceDetectorExecutable(data_provider,
                    fd_group, config.m_VariablesMap,
                    config.m_FaceDetectorDelay);
    face_detector_exec->m_Signal.connect(boost::bind(
            &MainModel::faces_observed, main_model, _1));
    face_detector_exec->start();
//    face_detector_exec->join();
#else
    ut_Timer * timer = new ut_Timer(
            boost::posix_time::milliseconds(config.m_FaceDetectorDelay));
    timer->m_Signal.connect(boost::bind(
            &MainModel::set_detect_faces_flag, main_model));
    timer->start();
#endif


    // Initialize main renderer to visualize the data and the results
//    MainRenderer * main_renderer = new MainRenderer(main_model);

    // Initialize main window to show the rendered images
    MainWindow * main_window;
    main_window = new MainWindow(*main_model);

    CvVideoWriter * video_writer;
    ModelDumperFactory * model_dumper_factory = ModelDumperFactory::instance();
    ModelDumper * model_dumper = 0;

    if (config.m_EnableVideoOutputFile) {
        float fps = data_provider->fps();
        if (fps <= 0) {
            fps = 6;
        }
        video_writer = cvCreateVideoWriter(config.m_OutputVideoFile.c_str(),
                CV_FOURCC('D', 'I', 'V', 'X'), fps,
                cvSize(data_provider->image_buffer()->width,
                       data_provider->image_buffer()->height),
                1);
        cout << "Writing video output to " << config.m_OutputVideoFile
             << ", size=(" << data_provider->image_buffer()->width
             << ", " << data_provider->image_buffer()->height << ")"
             << ", fps=" << fps << endl;
    }

    if (config.m_EnableDump) {
        model_dumper = model_dumper_factory->create_model_dumper(
                config.m_OutputDump, main_model);
        cout << "Dump to " << config.m_OutputDump << endl;
    }

    // Launch the main loop
    try {
        for (int i = 0; ; ++i) {
            #ifdef __ROSINTEGRATION_FOUND__
            ros::spinOnce();
            #endif
            //cout << "Updating main model..." << endl;
            main_model->update();
//            cout << "Updating main window..." << endl;
            const IplImage * rendering_results = main_window->update();
            if (!config.m_NoGui) {
                cvShowImage(MAIN_WINDOW_NAME.c_str(), rendering_results);
            }
            if (config.m_EnableVideoOutputFile) {
//                cout << "Writing frame " << i << endl;
                cvWriteFrame(video_writer, rendering_results);
            }
            if (config.m_EnableDump) {
                model_dumper->update();
            }
            cvWaitKey(10);

            #ifdef ENABLE_TRACKER_DUMP
            GLOBAL_FRAME_COUNTER++;
            #endif

        }
    } catch(...) {
        if (config.m_EnableVideoOutputFile) {
            cvReleaseVideoWriter(&video_writer);
        }
        if (config.m_EnableDump) {
            delete model_dumper; model_dumper = 0;
        }
        delete main_window;
    #ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
        delete face_detector_exec;
    #else
        delete timer;
    #endif
        delete main_model;
        delete fd_group;
        delete data_provider;
        throw;
        return 1;
    }

    if (config.m_EnableVideoOutputFile) {
        cvReleaseVideoWriter(&video_writer);
    }
    if (config.m_EnableDump) {
        delete model_dumper; model_dumper = 0;
    }
    delete main_window;
#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
    delete face_detector_exec;
#else
    delete timer;
#endif
    delete main_model;
    delete fd_group;
    delete data_provider;

    return 0;

} // main
