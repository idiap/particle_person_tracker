// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_VideoFileImageProvider - image provider using video file as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/filesystem/operations.hpp>   // file system
#include <sstream>                           // string streams
#include <iostream>                          // IO streams
#include <iomanip>

// LOCAL INCLUDES
#include "image_processing/ip_VideoFileImageProvider.h"
#include "image_processing/ip_Exceptions.h"

using namespace std;

namespace ImageProcessing {


ip_VideoFileImageProvider::
ip_VideoFileImageProvider(const string& device_name,
                          int width,
                          int height,
                          double scale,
                          int fps_pref,
                          bool offline_mode):
  ip_CvCaptureBasedImageProvider(width, height, scale),
  mFirstFrame(true),
  mOfflineMode(offline_mode)
{
  initialize_size(device_name);
  mFrameDelay_ms = 1000.0 / mFps;
  mFirstFrame = true;

  // Go back to the beginning og the video
  cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_POS_FRAMES, 0);
}


ip_VideoFileImageProvider::
~ip_VideoFileImageProvider()
{
}

void
ip_VideoFileImageProvider::
recompute_image(IplImage* buffer_image,
                const ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  int num_frames_to_skip = 0;

  if (!mOfflineMode) {
    if (!mFirstFrame) {
      boost::posix_time::time_duration time_delay = (time - mLastImageTimeStamp);
      long time_delay_ms = time_delay.total_milliseconds();
      num_frames_to_skip = static_cast<int>(time_delay_ms /
                                            mFrameDelay_ms + 0.5);
    }

    for(int i = 0; i<num_frames_to_skip-1; ++i)
      {
        cvQueryFrame(mCvCapture);
        mImageId++;
      }
  }

  //    cout << "next frame, ts=" << time << "; " <<
  //            "prev_ts=" << mLastImageTimeStamp << "; " <<
  //            "frame_delay=" << mFrameDelay_ms << "; " <<
  //            "current time=" << boost::posix_time::microsec_clock::local_time();
  //    cout << ", skipped " <<  num_frames_to_skip << " frames" << endl;

  IplImage *frame = cvQueryFrame(mCvCapture);

  // The first time, we don't increment: the first image is actually
  // index 0. In principle, recompute_image should be called before
  // image_id. Otherwise, for MOT challenge, there might be shifts
  if(!mFirstFrame) mImageId++; // if (mOfflineMode && !mFirstFrame) ???

  if (!frame) {
    throw ip_Exception("No more frames available!");
  }

  mFirstFrame = false;
  mLastImageTimeStamp = time;

  resize_or_copy(frame, buffer_image);

  // std::cout << name() << " get image " << mImageId << std::endl;
  // std::ostringstream oss;
  // oss << std::setfill('0') << std::setw(6) << mImageId << ".jpg";
  // cvSaveImage(oss.str().c_str(), frame);

  // cvReleaseImage(&frame);
}


CvCapture*
ip_VideoFileImageProvider::
initialize_capture(const std::string& device_name)
{
  CvCapture * capture = 0;

  namespace fs = boost::filesystem;

  ostringstream oss;
  capture = cvCaptureFromFile(device_name.c_str());

  if (!capture) {
    fs::path file_path(device_name.c_str());
    if (!fs::exists(file_path)) {
      oss << "File device: path " << device_name << " does not exist!";
      throw ip_Exception(oss.str());
    } else if (!fs::is_regular(file_path)) {
      oss << "File device: " << device_name << " is not a regular file!";
      throw ip_Exception(oss.str());
    } else {
      oss << "File device: error opening OpenCV file capture, codec problem?";
      throw ip_Exception(oss.str());
    }
  }
  return capture;
}

} // namespace ImageProcessing
