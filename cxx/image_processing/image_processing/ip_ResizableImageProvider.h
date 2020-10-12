/**
 * @brief Base class for provider that resize image in recompute_image
 *
 * Copyright (c) 2019-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */
#ifndef __IP_RESIZABLEIMAGEPROVIDER_H__
#define __IP_RESIZABLEIMAGEPROVIDER_H__

#include "image_processing/ip_ImageProvider.h"

namespace ImageProcessing {

class ip_ResizableImageProvider: public ip_ImageProvider
{
public:
  ip_ResizableImageProvider(int width, int height, double scale=1.0);
  virtual ~ip_ResizableImageProvider();
  virtual std::string name() const { return "[ip_ResizableImageProvider]"; }

protected:
  // virtual void initialize(const std::string& device_name="");
  void resize_or_copy(IplImage *frame, IplImage *buffer_image);

  /// Allocate memory
  virtual void initialize_size(const std::string& device_name) = 0;

  /// Compute the size at which images will be processed, using
  /// mScale, or keeping the aspect ration if eith height or width was
  /// provided.
  bool set_preferred_image_size();
  virtual int get_device_width() const = 0;
  virtual int get_device_height() const = 0;

protected:
  int mPreferredWidth;           // preferred width
  int mPreferredHeight;          // preferred height
  double mScale;                 // Whether to rescale image
  bool mResize;                  // should resize input images?

};



} // namespace ImageProcessing

#endif
