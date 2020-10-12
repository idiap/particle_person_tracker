/**
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief FaceTracker_ColourSegmentation - application to track faces
 * using segmentation into skin, hair, clothes, background based on
 * colour
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/thread/thread.hpp>                // boost threading

// PROJECT INCLUDES
#include <image_processing/ip_Exceptions.h>             // exceptions
#include <image_processing/ip_ImageProvider.h>          // image provider
#include <image_processing/ip_DataImageProvider.h>      // OpenCV image provider
#include <image_processing/ip_FaceDetectorExecutable.h> // face detector
#include <image_processing/ip_ImageProviderFactory.h>   // image provider factory
#include <utils/ut_Timer.h>                             // FD timer

// LOCAL INCLUDES
#include "FaceTrackerConfig.hpp"                  // config for face tracker
#include "MainModel_ColourSegmentationTracker.h"  // main model
#include "MainWindow_ColourSegmentationTracker.h" // main window
#include "ModelDumperCs_TA2Format.h"              // dumper

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace TTrackUtils;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const string MAIN_WINDOW_NAME = "CS Face Tracker Main Window";

/////////////////////////////// PUBLIC ///////////////////////////////////////

//#define USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
#define __RSB_OUTPUT_ENABLED__
//#define ENABLE_TRACKER_DUMP
#ifdef ENABLE_TRACKER_DUMP
extern int GLOBAL_FRAME_COUNTER;
#endif

int main (int argc, char **argv) {

    // Parse input arguments and config
    FaceTrackerConfig config;

    // Parse input arguments and config
    try {
        config.parse_command_line(argc, argv);
    } catch(FaceTracker_CmdLine_Parser_Exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    // Initialize data provider from the given device
//    ImageProcessing::ip_ImageProvider * data_provider =
//            new ip_DataImageProvider(config.m_VideoInputDevice,
//                    config.m_ImageWidth, config.m_ImageHeight);
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
        case 3: fdTypes.push_back(CVP_FACEDETECTOR_TORCH); break;
        case 4: fdTypes.push_back(CVP_FACEDETECTOR_RSB_FACE); break;
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
    cvp_FaceDetectorGroup * fd_group = new cvp_FaceDetectorGroup(fdTypes,
        config.m_VariablesMap);
    IplImage * example_image = data_provider->image_buffer();
    fd_group->prepare(example_image, 1.0);

    // Initialize main model using the data provider and config options
    MainModel_ColourSegmentationTracker * main_model =
            new MainModel_ColourSegmentationTracker(data_provider, fd_group,
                    config);

#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
    ip_FaceDetectorExecutable * face_detector_exec =
            new ip_FaceDetectorExecutable(data_provider,
                    fd_group, config.m_VariablesMap,
                    config.m_FaceDetectorDelay);
    face_detector_exec->m_Signal.connect(boost::bind(
            &MainModel_ColourSegmentationTracker::faces_observed,
                main_model, _1));
    face_detector_exec->start();
//    face_detector_exec->join();
#else
    ut_Timer * timer = new ut_Timer(
            boost::posix_time::milliseconds(config.m_FaceDetectorDelay));
    timer->m_Signal.connect(boost::bind(
            &MainModel_ColourSegmentationTracker::set_detect_faces_flag,
                main_model));
    timer->start();
#endif

    // Initialize main renderer to visualize the data and the results
//    MainRenderer * main_renderer = new MainRenderer(main_model);

    // Initialize main window to show the rendered images
    MainWindow_ColourSegmentationTracker * main_window;
    main_window = new MainWindow_ColourSegmentationTracker(*main_model);

    CvVideoWriter * video_writer;
    ModelDumperCs_TA2Format * model_dumper_ta2 = 0;

    if (config.m_EnableVideoOutputFile) {
        video_writer = cvCreateVideoWriter(config.m_OutputVideoFile.c_str(),
                CV_FOURCC('D', 'I', 'V', 'X'), data_provider->fps(),
                cvSize(data_provider->image_buffer()->width,
                       data_provider->image_buffer()->height),
                1);
    }

    if (config.m_EnableDump) {
        model_dumper_ta2 = new ModelDumperCs_TA2Format(
                config.m_OutputDump, main_model);
    }

    // Launch the main loop
    try {
        for (int i = 0; ; ++i) {
//            cout << "Updating main model... frame " << i << endl;
            main_model->update();
            //cout << "Updating main window..." << endl;
            if (!config.m_NoGui || config.m_EnableVideoOutputFile) {
                const IplImage * rendering_results = main_window->update();
                if (!config.m_NoGui) {
                    cvShowImage(MAIN_WINDOW_NAME.c_str(), rendering_results);
                }
                if (config.m_EnableVideoOutputFile) {
                    cvWriteFrame(video_writer, rendering_results);
                }
            }
            if (config.m_EnableDump) {
                model_dumper_ta2->update();
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
            delete model_dumper_ta2; model_dumper_ta2 = 0;
        }
#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
        delete face_detector_exec;
#else
        delete timer;
#endif
        delete main_window;
        return 1;
    }

    if (config.m_EnableVideoOutputFile) {
        cvReleaseVideoWriter(&video_writer);
    }
    if (config.m_EnableDump) {
        delete model_dumper_ta2; model_dumper_ta2 = 0;
    }
#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
    delete face_detector_exec;
#else
    delete timer;
#endif
    delete main_window;
    return 0;

} // main
