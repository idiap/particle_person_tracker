// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SingleImageProvider - image provider using image file as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/filesystem/operations.hpp>   // file system
#include <sstream>                           // string streams
#include <iostream>                          // IO streams

// LOCAL INCLUDES
#include <image_processing/ip_SingleImageProvider.h> // declaration of this
#include <image_processing/ip_Exceptions.h>  // exceptions

using namespace std;

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

static const int DEFAULT_FPS = 30;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_SingleImageProvider::
ip_SingleImageProvider(const string& file_name,
                       int width,
                       int height,
                       double scale,
                       int fps_pref,
                       bool offline):
  mPreferredWidth(width),
  mPreferredHeight(height),
  mScale(scale),
  mResize(true)
{
  mFileImage = cvLoadImage(file_name.c_str());

  m_Fps = (fps_pref == -1 ? DEFAULT_FPS : fps_pref);
  mFrameDelay_ms = 1000.0 / m_Fps;

  if (mFileImage) {
    set_preferred_image_size();
    mResize = (mPreferredWidth != mFileImage->width) ||
      (mPreferredHeight != mFileImage->height);
    IplImage * buffer_image = cvCreateImage(cvSize(mPreferredWidth,
                                                   mPreferredHeight),
                                            mFileImage->depth,
                                            mFileImage->nChannels);
    image(buffer_image);
  }
}


ip_SingleImageProvider::
ip_SingleImageProvider(const IplImage * source_image,
                       int width,
                       int height,
                       double scale,
                       int fps_pref,
                       bool offline):
  mPreferredWidth(width),
  mPreferredHeight(height),
  mScale(scale),
  mResize(true)
{
  mFileImage = cvCloneImage(source_image);

  m_Fps = (fps_pref == -1 ? DEFAULT_FPS : fps_pref);
  mFrameDelay_ms = 1000.0 / m_Fps;

  if (mFileImage) {
    set_preferred_image_size();
    mResize = (mPreferredWidth != mFileImage->width) ||
      (mPreferredHeight != mFileImage->height);
    IplImage * buffer_image = cvCreateImage(cvSize(mPreferredWidth,
                                                   mPreferredHeight),
                                            mFileImage->depth,
                                            mFileImage->nChannels);
    image(buffer_image);
  }
} // ip_SingleImageProvider

/* virtual */ ip_SingleImageProvider::~ip_SingleImageProvider() {
    if (mFileImage) {
        cvReleaseImage(&mFileImage);
    }
} // ~ip_SingleImageProvider

/* virtual */ float
ip_SingleImageProvider::fps() const {
    return m_Fps;
} // fps

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_SingleImageProvider::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

    if (!mFileImage) {
        throw ip_Exception("No more frames available!");
    }

    try{
        if (mResize) {
            cvResize(mFileImage, buffer_image, CV_INTER_LINEAR);
        } else {
            cvCopy(mFileImage, buffer_image);
        }
    } catch (...) {
        cout << "Exception caught in video file data provider!" << endl << flush;
        cout << "File image: " << mFileImage << ", ("
             << mFileImage->width << ", " << mFileImage->height << "), "
             << "depth=" << mFileImage->depth << ", "
             << "nChannels=" << mFileImage->nChannels << endl;
        cout << "Buffer image: " << buffer_image << ", ("
             << buffer_image->width << ", " << buffer_image->height << "), "
             << "depth=" << buffer_image->depth << ", "
             << "nChannels=" << buffer_image->nChannels << endl;

    }

} // recompute_image

/////////////////////////////// PRIVATE //////////////////////////////////////

bool
ip_SingleImageProvider::
set_preferred_image_size()
{

  int device_width = mFileImage->width;
  int device_height = mFileImage->height;

  std::cout << "[SingleImageProvider] Image dimensions "
            << device_width << "x" << device_height
            << std::endl;

  if (mPreferredWidth == -1) {
    if (mPreferredHeight == -1) {
      mPreferredWidth = device_width;
      mPreferredHeight = device_height;
    } else {
      mPreferredWidth = mPreferredHeight * device_width / device_height;
    }
  } else if (mPreferredHeight == -1) {
    mPreferredHeight = mPreferredWidth * device_height / device_width;
  }

  mPreferredWidth = static_cast<int>(mPreferredWidth*mScale);
  mPreferredHeight = static_cast<int>(mPreferredHeight*mScale);

  std::cout << "[SingleImageProvider] Selected dimensions "
            << mPreferredWidth << "x" << mPreferredHeight
            << std::endl;

  return true;
}

} // namespace ImageProcessing
