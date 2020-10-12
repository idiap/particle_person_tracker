/**
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * @brief application to track faces, estimate head poses and the
 * associated visual foci of attention
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include <omp.h>

#ifdef __ROSINTEGRATION_FOUND__
#include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
#endif

#include <image_processing/ip_Exceptions.h>
#include <image_processing/ip_ImageProvider.h>
#include <image_processing/ip_DataImageProvider.h>
#include <image_processing/ip_FaceDetectorExecutable.h>
#include <image_processing/ip_ImageProviderFactory.h>

#include <utils/ut_Timer.h>

#include "FaceTrackerConfig.hpp"
#include "MainModel.h"
#include "MainWindow.h"
#include "ModelDumperFactory.h"

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace TTrackUtils;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const string MAIN_WINDOW_NAME = "Idiap Visual Tracker";


// #define USE_SEPARATE_THREAD_FOR_FACE_DETECTORS

//#define ENABLE_TRACKER_DUMP
#ifdef ENABLE_TRACKER_DUMP
extern int GLOBAL_FRAME_COUNTER;
#endif

int main (int argc, char **argv)
{
  // omp_set_num_threads(8);
  // std::cout << "max nb " << omp_get_max_threads() << std::endl;
  // for(int i=0 ; i<argc ; ++i) std::cout << argv[i] << std::endl;

  // Parse input arguments and config
  FaceTrackerConfig config;
  try
    {
      config.parse_command_line(argc, argv);
    }
  catch(FaceTracker_CmdLine_Parser_Exception& e)
    {
      cerr << e.what() << endl;
      return 1;
    }

  if(config.m_Visu > 0)
    {
      cvNamedWindow(MAIN_WINDOW_NAME.c_str(),
                    CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
      // const int kw = 480;
      // const int kh = 260;
      // cvResizeWindow(MAIN_WINDOW_NAME.c_str(), 4*kw, 4*kh);
      // cvMoveWindow(MAIN_WINDOW_NAME.c_str(), 4*kw, 0);
    }

#ifdef __ROSINTEGRATION_FOUND__
  ros::init(argc, argv, "facetrack");
#endif

  // Initialize data provider from the given device
  ImageProcessing::ip_ImageProvider *data_provider = 0;
  try
    {
      data_provider = ip_ImageProviderFactory::instance()->
        create_image_provider(config.m_VideoInputDevice,
                              config.m_ImageWidth,
                              config.m_ImageHeight,
                              config.m_Scale,
                              config.m_Fps,
                              !(config.m_Online));
  }
  catch(ImageProcessing::ip_Exception& e)
    {
      cerr << e.what() << endl;
      return 1;
    }


  std::cout << "[OpenPoseTracker] Source "
            << config.m_VideoInputDevice << std::endl;
  std::cout << "[OpenPoseTracker] Type "
            << str(data_provider->id()) << std::endl;
  std::cout << "[OpenPoseTracker] Number of particles "
            << config.m_NumTrackerParticles << std::endl;
  std::cout << "[OpenPoseTracker] Minimum face height "
            << config.m_MinFaceHeight << std::endl;

  // Prepare face detector types based on config
  std::list<cvp_FaceDetectorType> fdTypes;
  switch(config.m_FaceDetectorType)
    {
    case CVP_FACEDETECTOR_OPENCV_FACE:
      fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_FACE);
      break;
    case CVP_FACEDETECTOR_CTU:
      fdTypes.push_back(CVP_FACEDETECTOR_CTU);
      break;
    // case CVP_FACEDETECTOR_TORCH:
    //   fdTypes.push_back(CVP_FACEDETECTOR_TORCH);
    //   break;
    case CVP_FACEDETECTOR_OPENPOSE:
      fdTypes.push_back(CVP_FACEDETECTOR_OPENPOSE);
      break;
    case CVP_FACEDETECTOR_OPENHEADPOSE:
      fdTypes.push_back(CVP_FACEDETECTOR_OPENHEADPOSE);
      break;
    default:
      fdTypes.push_back(CVP_FACEDETECTOR_UNKNOWN);
      break;
    }
  if (config.m_UseProfiles){
    switch(config.m_FaceDetectorType)
      {
      // case CVP_FACEDETECTOR_TORCH:
      //   fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_PROFILE_LEFT);
      //   fdTypes.push_back(CVP_FACEDETECTOR_OPENCV_PROFILE_RIGHT);
      //   break;
    }
  }

  // Construct face detector group from the provided types
  cvp_FaceDetectorGroup *fd_group =
    new cvp_FaceDetectorGroup(fdTypes, config.m_VariablesMap);

  // std::cout << "Get image" << std::endl;
  // IplImage *example_image = data_provider->image_buffer();
  // std::cout << "example_image " << example_image << std::endl;
  // std::cout << "example_image->width() " << example_image->width << std::endl;
  // std::cout << "Prepare fd group" << std::endl;
  // fd_group->prepare(example_image, 1.0);

  MainModel *main_model = new MainModel(data_provider, fd_group, config);

#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
  ip_FaceDetectorExecutable *face_detector_exec =
    new ip_FaceDetectorExecutable(data_provider,
                                  fd_group,
                                  config.m_VariablesMap,
                                  config.m_FaceDetectorDelay);
  face_detector_exec->m_Signal.connect(boost::bind(&MainModel::faces_observed,
                                                   main_model, _1));
  face_detector_exec->start();
#else
  ut_Timer *timer =
    new ut_Timer(boost::posix_time::milliseconds(config.m_FaceDetectorDelay));
  timer->m_Signal.connect(boost::bind(&MainModel::set_detect_faces_flag, main_model));
  timer->start();
#endif

  // Initialize main window to show the rendered images
  MainWindow *main_window = new MainWindow(*main_model);

  ModelDumperFactory *model_dumper_factory = ModelDumperFactory::instance();
  ModelDumper *model_dumper = 0;

  if(config.m_EnableVideoOutputFile)
    {
      std::string dir = config.m_OutputVideoFile;
      if(!boost::filesystem::is_directory(dir) &&
         !boost::filesystem::create_directory(dir))
        {
          std::cout << "Cannot make directory " << dir << std::endl;
          return 1;
        }
    }

  if(config.m_EnableDump)
    {
      model_dumper =
        model_dumper_factory->create_model_dumper(config.m_OutputDump,
                                                  main_model);
      std::cout << "[OpenPoseTracker] Output dump " << config.m_OutputDump << std::endl;
    }

  // Launch the main loop
  int i = 0;
  try {
#ifdef __ROSINTEGRATION_FOUND__
    while (ros::ok())
#else
    for (i=0; ; ++i)
#endif
      {
        bool success = main_model->update();
        // main_model->update();

        if(!success) break;

        const IplImage *rendering_results = main_window->update();

        // std::cout << "i " << i << " time " << data_provider->time() << std::endl;
        // std::cout << "rendering_results " << rendering_results << std::endl;

        // if(rendering_results)
        //   {
        //     std::cout << "rendering_results.width " << rendering_results->width << std::endl;
        //   }
        // If the image size is not known yet, the image buffer is
        // empty, so we continue until the first frame is received.
        // if(!rendering_results)
        //   {
        //     std::cout << "No image yet to render" << std::endl;
        //     continue;
        //   }

        // if(config.m_EnableVideoOutputFile && i>1 && rendering_results)
        if(config.m_EnableVideoOutputFile && rendering_results)
          {
            uint64_t image_id = 0; // Can be really big (nanosec)

#ifdef __ROSINTEGRATION_FOUND__
            image_id = data_provider->get_data_time_ns();
#else
            image_id = static_cast<uint64_t>(data_provider->image_id());
#endif
            std::ostringstream oss;
            oss << std::setfill('0') << std::setw(6) << image_id << ".jpg";

            boost::filesystem::path p(config.m_OutputVideoFile);
            p /= oss.str();
            // std::cout << "Saving to " << p.string() << std::endl;
            // cvSaveImage(p.string().c_str(), rendering_results);
            cv::imwrite(p.string().c_str(), cv::cvarrToMat(rendering_results));
            // cvSaveImage(p.string().c_str(), rendering_results);

            // if(0)
            //   {
            //     // fvw << "touch -d \"$(date +\"%Y-%m-%d %H:%M:%S.%N\" -d @\""
            //     //     << std::fixed << ms << "\")\" "
            //     //     << oss.str() << std::endl;
            //   }
          }

        if(config.m_Visu > 0 && rendering_results)
          {
            cvShowImage(MAIN_WINDOW_NAME.c_str(), rendering_results);
          }

        if(config.m_EnableDump && rendering_results)
          {
            model_dumper->update();
          }

        cvWaitKey(30);

        // unsigned char key = static_cast<unsigned char>(cvWaitKey(30));
        // Remove for now because we don't want to break that node by
        // mistake. Use __ROSINTEGRATION_FOUND__ ???]
        //
        // if(key=='q')
        //   {
        //     std::cout << "Breaking" << std::endl;
        //     break;
        //   }

#ifdef __ROSINTEGRATION_FOUND__
        ros::spinOnce();
        ++i;
#endif

#ifdef ENABLE_TRACKER_DUMP
        GLOBAL_FRAME_COUNTER++;
#endif

      }
  }
  // catch(...)
  catch (const std::exception &exc)
    {
      std::cout << "Caught exception" << std::endl;
      std::cout << exc.what() << std::endl;
      // if(vw && vw->isOpened()) { delete vw; vw = 0; }

//       if (config.m_EnableDump) {
//         delete model_dumper; model_dumper = 0;
//       }
//       delete main_window;
      // return 1;
  }

  // if(vw && vw->isOpened()) delete vw;
//   if (config.m_EnableVideoOutputFile) {
//     cvReleaseVideoWriter(&video_writer);
//   }
//   if (config.m_EnableDump) {
//     delete model_dumper; model_dumper = 0;
//   }

#ifdef USE_SEPARATE_THREAD_FOR_FACE_DETECTORS
  delete face_detector_exec;
// #else
//   delete timer; // FIXME: signals2 breaks in destructor
#endif
  // delete main_model; // FIXME: PimFeature break in destructor...
  delete fd_group;
  delete data_provider;
  delete main_window;
  if(model_dumper) delete model_dumper;

  return 0;
}
