/**
 * @file cxx/image_processing/src/ip_WebcamImageProvider.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image provider for web cameras
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_WEBCAMIMAGEPROVIDER_H__
#define __IP_WEBCAMIMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <string>                    // STL string

// LOCAL INCLUDES
#include "image_processing/ip_CvCaptureBasedImageProvider.h"

namespace ImageProcessing {

/// @brief Class to represent an image provider for webcam data
///
/// This class defines an image provider that uses a web camera as data source.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    27.11.2011

class ip_WebcamImageProvider: public ip_CvCaptureBasedImageProvider
{
public:
  // LIFECYCLE

  /// Constructor to initialize webcam image provider
  /// @param width Preferred image width, -1 for the default device width
  /// @param height Preferred image height, -1 for the default device height
  ip_WebcamImageProvider(int width = -1,
                         int height = -1,
                         double scale = 1.0);

  /// Destructor, deallocates capture if it was allocated
  virtual ~ip_WebcamImageProvider();
  virtual std::string name() { return "[ip_WebcamImageProvider]"; }

  /// Overrides base class method, returns IP_IMG_PROVIDER_WEBCAM_DATA
  /// @return Provider's ID
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_WEBCAM_DATA; }

protected:
    /// Overrides base class method.
    /// Obtains the most recent image from the web camera,
    /// writes its part corresponding to the provided ROI to the buffer_image.
    /// Receives the acquisition time of the previous image in time parameter
    /// and fills it with the current time.
    /// @param image Image to write results to
    /// @param roi Region of interest of a source image to be copied into the buffer
    /// @param time Previous image timestamp, to be filled with current timestamp
    virtual void recompute_image(IplImage* image,
                                 const ip_RoiWindow& roi,
                                 boost::posix_time::ptime& time);

protected:
    // attempts to initialize capture using the device name
    virtual CvCapture* initialize_capture(const std::string& device_name = "0");

private:


    // attempts to make the device stream images of the preferred size
    // returns true if succeeded
    // bool set_preferred_image_size();

    // CvCapture *mCvCapture;        // OpenCV capture handle
    boost::gregorian::date mDate;  // date to remember capture start

};

} // namespace ImageProcessing

#endif // __IP_WEBCAMIMAGEPROVIDER_H__
