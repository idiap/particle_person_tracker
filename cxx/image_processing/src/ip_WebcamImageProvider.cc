// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_WebcamImageProvider - image provider using web camera as a data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#include <boost/filesystem/operations.hpp> // file system
#include <sstream>                         // string streams
#include <iostream>                        // IO streams

#include "image_processing/ip_WebcamImageProvider.h"
#include "image_processing/ip_Exceptions.h"

using namespace std;

namespace ImageProcessing {

ip_WebcamImageProvider::
ip_WebcamImageProvider(int width, int height, double scale):
  ip_CvCaptureBasedImageProvider(width, height, scale)
{
  initialize_size();
  mDate = boost::gregorian::day_clock::local_day();
}


ip_WebcamImageProvider::
~ip_WebcamImageProvider()
{
}

void
ip_WebcamImageProvider::
recompute_image(IplImage *buffer_image,
                const ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  IplImage *frame = cvQueryFrame(mCvCapture);
  mImageId++;
  time = boost::posix_time::microsec_clock::local_time();

  if(!frame)
    throw ip_Exception("No more frames available!");

  resize_or_copy(frame, buffer_image);

  // try{
  //   if(mResize)
  //     {
  //       cvResize(frame, buffer_image, CV_INTER_LINEAR);
  //     }
  //   else
  //     {
  //       cvCopy(frame, buffer_image);
  //     }
  // }
  // catch (...)
  //   {
  //     cout << "Exception caught in data provider" << endl << flush;
  //   }
}


CvCapture*
ip_WebcamImageProvider::
initialize_capture(const std::string& device_name)
{
  CvCapture *capture = 0;

  ostringstream oss;
  try {
    capture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!capture) {
      oss << " error opening OpenCV camera capture!";
      throw ip_Exception(oss.str());
    }
    return capture;
  } catch (ip_Exception& e) {
    oss << "Camera device: " << e.what() << endl;
  }

  if (!capture) {
    throw ip_Exception(oss.str());
  }
  return capture;
}

// bool
// ip_WebcamImageProvider::
// set_preferred_image_size()
// {
//   bool result = true;

//   int device_width =
//     static_cast<int>(cvGetCaptureProperty(mCvCapture,
//                                           CV_CAP_PROP_FRAME_WIDTH));
//   int device_height =
//     static_cast<int>(cvGetCaptureProperty(mCvCapture,
//                                           CV_CAP_PROP_FRAME_HEIGHT));

//   std::cout << "[WebcamImageProvider] Device dimensions "
//             << device_width << "x" << device_height
//             << std::endl;

//   if (mPreferredWidth == -1) {
//     if (mPreferredHeight == -1) {
//       mPreferredWidth = device_width;
//       mPreferredHeight = device_height;
//     } else {
//       mPreferredWidth = mPreferredHeight * device_width / device_height;
//     }
//   } else if (mPreferredHeight == -1) {
//     mPreferredHeight = mPreferredWidth * device_height / device_width;
//   }

//   mPreferredWidth = static_cast<int>(mPreferredWidth*mScale);
//   mPreferredHeight = static_cast<int>(mPreferredHeight*mScale);

//   if (result && (device_width != mPreferredWidth)) {
//     result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_WIDTH,
//                                   mPreferredWidth);
//   }

//   if (result && (device_height != mPreferredHeight)) {
//     result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_HEIGHT,
//                                   mPreferredHeight);
//   }

//   std::cout << "[WebcamImageProvider] Selected dimensions "
//             << mPreferredWidth << "x" << mPreferredHeight
//             << std::endl;

//   // According to the documentation of OpenCV, return value can be
//   // true even if the device does not support setting a different size
//   // so we check again
//   device_width =
//     static_cast<int>(cvGetCaptureProperty(mCvCapture,
//                                           CV_CAP_PROP_FRAME_WIDTH));
//   device_height =
//     static_cast<int>(cvGetCaptureProperty(mCvCapture,
//                                           CV_CAP_PROP_FRAME_HEIGHT));

//   if(device_height != mPreferredHeight || device_width != mPreferredWidth)
//     {
//       std::cout << "[WebcamImageProvider] Device does not support "
//         "setting property. Resize will be manual" << std::endl;
//       result = false;
//     }

//   return result;
// }

} // namespace ImageProcessing
