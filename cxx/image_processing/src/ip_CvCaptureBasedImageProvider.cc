/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#include "image_processing/ip_CvCaptureBasedImageProvider.h"

namespace ImageProcessing {

ip_CvCaptureBasedImageProvider::
ip_CvCaptureBasedImageProvider(int width,
                               int height,
                               double scale):
  ip_ResizableImageProvider(width, height, scale),
  mCvCapture(NULL),
  mImageId(0),
  mFps(-1)
{
}


ip_CvCaptureBasedImageProvider::
~ip_CvCaptureBasedImageProvider()
{
  if(mCvCapture)
    {
      cvReleaseCapture(&mCvCapture);
      mCvCapture = 0;
    }
}


int
ip_CvCaptureBasedImageProvider::
get_device_width() const
{
  return static_cast<int>(cvGetCaptureProperty(mCvCapture,
                                               CV_CAP_PROP_FRAME_WIDTH));
}

int
ip_CvCaptureBasedImageProvider::
get_device_height() const
{
  return static_cast<int>(cvGetCaptureProperty(mCvCapture,
                                               CV_CAP_PROP_FRAME_HEIGHT));
}


void
ip_CvCaptureBasedImageProvider::
initialize_size(const std::string& device_name)
{
  mCvCapture = initialize_capture(device_name);
  mFps = cvGetCaptureProperty(mCvCapture, CV_CAP_PROP_FPS);
  mResize = !set_preferred_image_size();

  IplImage *frame = cvQueryFrame(mCvCapture);
  // cvSaveImage("first.jpg", frame);
  IplImage *buffer_image = cvCreateImage(cvSize(mPreferredWidth,
                                                mPreferredHeight),
                                         frame->depth,
                                         frame->nChannels);
  image(buffer_image);
}




} // namespace ImageProcessing
