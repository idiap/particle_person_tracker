// Copyright) 2011 Idiap Research Institute
//
// ip_VideoFileImageProvider - image provider using video file as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_VIDEOFILEIMAGEPROVIDER_H__
#define __IP_VIDEOFILEIMAGEPROVIDER_H__

#include <string>
#include <highgui.h>

#include "image_processing/ip_CvCaptureBasedImageProvider.h"

namespace ImageProcessing {

/// @brief Class to represent an image provider for a video file
///
/// This class defines an image provider that uses a video file as data source.
/// Two modes are available for streaming the data:
///     offline (all frames are processed one by one,
///              processing time is not taken into account),
///     online  (skips frames according to time elapsed between
///              the two subsequent requests,
///     uses information on frame rate from the video file).
/// OpenCV is used to read video files and information on frame rate.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    28.11.2011

class ip_VideoFileImageProvider: public ip_CvCaptureBasedImageProvider
{
public:
  /// Constructor to initialize data image provider
  /// @param device_name Input file name
  /// @param width        Preferred image width, -1 for the default device width
  /// @param height       Preferred image height, -1 for the default device height
  /// @param fps          Preferred fps rate, -1 for the default fps rate
  /// @param offline_mode Flag indicating the mode (online/offline)
  ///                          in which the provider is launched.
  ///                     Queries every frame in offline mode, skips as much
  ///                     frames as required to match the elapsed processing
  ///                     time in online mode
  ip_VideoFileImageProvider(const std::string& device_name,
                            int width = -1,
                            int height = -1,
                            double scale = 1,
                            int fps = -1,
                            bool offline_mode = false);

  /// Destructor, deallocates capture if it was allocated
  virtual ~ip_VideoFileImageProvider();
  virtual std::string name() { return "[ip_VideoFileImageProvider]"; }

  /// Overrides base class method, returns IP_IMG_PROVIDER_FILE_DATA
  /// @return Provider's ID: IP_IMG_PROVIDER_FILE_DATA
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_FILE_DATA; }

protected:
  /// Overrides base class method.
  /// Obtains the most recent image from the video file.
  /// Prepares either the next frame (offline mode) or uses
  /// the delay between current time and the time when a frame was
  /// last requested to skip the corresponding number of frames (online mode)
  /// @param image Image to write results to
  /// @param roi   Region of interest for which to recompute the image,
  ///              currently ignored
  /// @param time  Timestamp of the previous image is provided as an input,
  ///              fills current timestamp after processing as an output
  virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
                               boost::posix_time::ptime& time);

protected:
  // Attempts to initialize capture using the device name (path to a video file),
  // throws ip_Exception on error
  // (file does not exist, not a file, video codec not found, etc.)
  virtual CvCapture* initialize_capture(const std::string& device_name);

private:
  bool mFirstFrame;          // current frame is the first one?
  const bool mOfflineMode;   // process every frame if true
  boost::posix_time::ptime mLastImageTimeStamp;
  float mFrameDelay_ms;      // time delay between two consecutive frames

};

} // namespace ImageProcessing

#endif // __IP_VIDEOFILEIMAGEPROVIDER_H__
