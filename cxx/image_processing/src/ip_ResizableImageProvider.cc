/*
 * Copyright (c) 2019-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#include "image_processing/ip_ResizableImageProvider.h"


namespace ImageProcessing {


ip_ResizableImageProvider::
ip_ResizableImageProvider(int width,
                          int height,
                          double scale):
  mPreferredWidth(width),
  mPreferredHeight(height),
  mScale(scale),
  mResize(false)
{
}


ip_ResizableImageProvider::
~ip_ResizableImageProvider()
{
}


bool
ip_ResizableImageProvider::
set_preferred_image_size()
{
  bool result = true;

  int device_width = get_device_width();
  int device_height = get_device_height();

  mOriginalWidth = device_width;
  mOriginalHeight = device_height;

  std::cout << name() << " Device dimensions "
            << device_width << "x" << device_height
            << std::endl;

  if(mPreferredWidth == -1)
    {
    if(mPreferredHeight == -1)
      {
        mPreferredWidth = device_width;
        mPreferredHeight = device_height;
      }
    else
      {
        mPreferredWidth = mPreferredHeight * device_width / device_height;
      }
    }
  else if(mPreferredHeight == -1)
    {
      mPreferredHeight = mPreferredWidth * device_height / device_width;
    }

  mPreferredWidth = static_cast<int>(mPreferredWidth*mScale);
  mPreferredHeight = static_cast<int>(mPreferredHeight*mScale);

  // if(result && (device_width != mPreferredWidth))
  //   {
  //     result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_WIDTH,
  //                                   mPreferredWidth);
  // }

  // if(result && (device_height != mPreferredHeight))
  //   {
  //     result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_HEIGHT,
  //                                   mPreferredHeight);
  //   }

  std::cout << name() << " Selected dimensions "
            << mPreferredWidth << "x" << mPreferredHeight
            << std::endl;

  result = false;
  // According to the documentation of OpenCV, return value can be
  // true even if the device does not support setting a different size
  // so we check again
  // device_width = get_device_width();
  // device_height = get_device_height();

  // if(device_height != mPreferredHeight || device_width != mPreferredWidth)
  //   {
  //     std::cout << name() << " Device does not support setting property."
  //               << std::endl;
  //     result = false;
  //   }

  return result;
}


void
ip_ResizableImageProvider::
resize_or_copy(IplImage *frame, IplImage *buffer_image)
{
  try
    {
      if(mResize)
        {
          cvResize(frame, buffer_image, CV_INTER_LINEAR);
        }
      else
        {
          cvCopy(frame, buffer_image);
        }
    }
  catch(...)
    {
      std::cout << name() << "Exception caught in provider" << std::endl;
    }
}


} // namespace ImageProcessing
