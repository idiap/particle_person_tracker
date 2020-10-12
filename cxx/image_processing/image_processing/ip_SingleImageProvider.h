// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SingleImageProvider - image provider using image file as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SINGLEIMAGEPROVIDER_H__
#define __IP_SINGLEIMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <highgui.h>                 // for CvCapture

// LOCAL INCLUDES
#include "image_processing/ip_ImageProvider.h"        // base class

namespace ImageProcessing {

/// @brief Class to represent an image provider for an image file
///
/// This class defines an image provider that uses an image file as data source.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    28.11.2011

class ip_SingleImageProvider: public ip_ImageProvider
{
public:

  /// Constructor. Initializes single image provider with an image loaded
  /// from the file system.
  /// @param file_name Input image file name
  /// @param width Preferred image width, -1 to keep the original image width
  /// @param height Preferred image height, -1 to keep the original image
  /// height
  /// @param fps Preferred frame rate, -1 for the default rate (30 FPS)
  /// @param offline Offline mode flag, TRUE by default
  ip_SingleImageProvider(const std::string& file_name,
                         int width = -1,
                         int height = -1,
                         double scale = 1,
                         int fps = -1,
                         bool offline = true);

  /// Constructor. Initializes single image provider with a provided image.
  /// @param image Input image
  /// @param width Preferred image width, -1 to keep the original image width
  /// @param height Preferred image height, -1 to keep the original image
  /// height
  /// @param fps Preferred frame rate, -1 for the default fps rate (30 FPS)
  /// @param offline Offline mode flag, TRUE by default
  ip_SingleImageProvider(const IplImage * image,
                         int width = -1,
                         int height = -1,
                         double scale = 1.0,
                         int fps = -1,
                         bool offline = true);

  /// Destructor, deallocates image if it was loaded
  virtual ~ip_SingleImageProvider();

  /// Overrides base class method, returns IP_IMG_PROVIDER_DATA by default
  /// @return Provider's ID
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_IMAGE_DATA; };

  /// Overrides base class method, returns the FPS (frames per second) rate
  /// @return FPS (frames per second) rate.
  virtual float fps() const;

protected:

  /// Overrides base class method, obtains the most recent image
  /// @param image Image to write results to
  virtual void recompute_image(IplImage* image,
                               const ip_RoiWindow& roi,
                               boost::posix_time::ptime& time);

private:

  // attempts to make the device stream images of the preferred size
  // returns true if succeeded
  bool set_preferred_image_size();

  IplImage * mFileImage;         // image loaded from file
  int mPreferredWidth;           // preferred width
  int mPreferredHeight;          // preferred height
  double mScale;
  bool mResize;                  // should resize input images?
  float mFrameDelay_ms;          // time delay between two consecutive frames
  float m_Fps;                   // FPS rate
};

} // namespace ImageProcessing

#endif // __IP_SINGLEIMAGEPROVIDER_H__
