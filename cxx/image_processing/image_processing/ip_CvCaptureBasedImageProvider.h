/**
 * @brief Base class for provider based on cvcapture
 *
 * Copyright (c) 2019-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */
#ifndef __IP_CVCAPTUREBASEDIMAGEPROVIDER_H__
#define __IP_CVCAPTUREBASEDIMAGEPROVIDER_H__

#include <string>
#include <highgui.h>
#include "image_processing/ip_ResizableImageProvider.h"

namespace ImageProcessing {

class ip_CvCaptureBasedImageProvider: public ip_ResizableImageProvider
{
public:
  ip_CvCaptureBasedImageProvider(int width, int height, double scale=1.0);
  virtual ~ip_CvCaptureBasedImageProvider();
  virtual std::string name() { return "[ip_CvCaptureBasedImageProvider]"; }

  virtual unsigned image_id() const { return mImageId; }
  virtual float fps() const { return mFps; }

protected:
  virtual int get_device_width() const;
  virtual int get_device_height() const;
  virtual void initialize_size(const std::string& device_name="");

  /// Attempts to initialize capture using the device name
  virtual CvCapture* initialize_capture(const std::string& device_name="") = 0;


protected:
  CvCapture* mCvCapture;         // OpenCV capture handle
  unsigned mImageId;
  float mFps;                    // FPS rate

};

} // namespace ImageProcessing

#endif
