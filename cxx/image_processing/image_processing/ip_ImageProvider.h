/**
 * @file cxx/image_processing/image_processing/ip_ImageProvider.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Representation of an abstract image provider independent of image
 *        source
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_IMAGEPROVIDER_H__
#define __IP_IMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <cv.h>                                    // for IplImage from OpenCV
#include <boost/thread/shared_mutex.hpp>           // boost shared mutex
#include "boost/date_time/posix_time/posix_time.hpp" // boost posix time

// LOCAL INCLUDES
#include "ip_ImageProviderType.h"         // provider types
#include "ip_RoiWindow.h"                 // ROI window

namespace ImageProcessing {

/// @brief Class to represent an abstract image provider
///
/// This class defines a common interface for components that generate results
/// in the form of an image.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_ImageProvider
{

public:

  // LIFECYCLE

  /// Default constructor to initialize empty provider
  ip_ImageProvider();

  /// Destructor, deallocates image buffer if it was allocated
  virtual ~ip_ImageProvider();

  virtual std::string name() { return "[ip_ImageProvider]"; }
  // OPERATIONS

  /// Return current image, may force recalculation
  /// @return Image of currently computed results
  // IplImage* image();

  /// Return current image, may force recalculation.
  /// Use this method with precaution:
  /// if several components use the same provider, it might be undesirable
  /// to recompute the image locally.
  /// @param roi Region of interest to consider if recalculation is performed
  /// @return Image of currently computed results
  IplImage* image(const ip_RoiWindow& roi = ip_RoiWindow());

  /// Current image ID. Image ID is its 1-based index in the original
  /// sequence. In offline mode the difference of IDs of two consecutive
  /// images is 1. In online mode this is not guaranteed, the ID refers to
  /// the original recording index, not to the actual playback index.
  /// However, sources that do not work in offline mode (like webcams)
  /// have their image ID related to the playback image number.
  ///
  /// @return Current image ID
  virtual unsigned image_id() const;

  /// Current image timestamp.
  /// @return Current image timestamp
  boost::posix_time::ptime time() const;

  /// Returns image provider FPS (frames per second) rate.
  /// This rate can be constant and predefined
  /// (for video files, image file lists) or variable (for online camera
  /// capture systems). FPS rate is not defined for intermediate image
  /// providers (e.g. gradient image computation).
  /// Returns -1 if the FPS rate cannot be evaluated (e.g., for online
  /// camera capture when capturing has not yet been started).
  /// @return Image provider FPS (frames per second) rate.
  virtual float fps() const;

  /// Thread-safe method to copy provider image to any destination buffer.
  /// This method performs full image buffer copy protected by a mutex.
  /// It should not be used in single-threaded applications,
  /// since it introduces the overhead of the copying an image.
  /// It is required for multithreaded applications
  /// where one needs to guarantee that while the image is accessed
  /// for read operations, it would not be overwritten.
  /// @param dest_image Destination buffer
  /// @param time Timestamp of the buffer being copied
  void copy_image(IplImage* dest_image, boost::posix_time::ptime& time);

  /// Return image buffer without recalculation
  /// @return Image buffer
  IplImage* image_buffer() const;

  /// Makes current image invalid, forces results recalculation
  virtual void invalidate();

  /// Return provider's ID, IP_IMG_PROVIDER_UNKNOWN by default
  /// @return Provider's ID
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_UNKNOWN; };

  /// For ROS messages, return the header.frame_id value. This
  /// function needs to be implemented in derive classes based on ROS
  virtual std::string get_tf_frame() { return ""; }

  /// For ROS messages, return the header.stamp in ns
  // virtual uint64_t get_ros_time_ns() { return 0; }

  /// Return the time of the data which is processed. This is useful
  /// when saving results, to have access to the correct frame ID.
  ///
  ///   - For ROS messages, return the header.stamp in ns
  ///   - For FileList, return the "int" file name if applicable
  ///   - TODO: For Video, return the timestamp when provided
  ///     in a separate text file
  ///
  virtual uint64_t get_data_time_ns() { return mDataTime; }

  /// Indicate if the model should continue trying grabbing frame if
  /// attempt was unscucessful.
  ///
  /// For instance, for ip_FileListImageProvider,
  /// ip_VideoFileImageProvider, the return value is false because
  /// this correspond to the end to the sequence.
  ///
  /// For ip_RosImageProvider, this is true, because an unsucessful
  /// attempt can mean that the messages have not arrived yet, or that
  /// the camera is down for a moment, etc. So the model should
  /// continue grabbing frame
  virtual bool should_wait() const { return false; }

  int get_original_width() const { return mOriginalWidth; }
  int get_original_height() const { return mOriginalHeight; }

protected:

  // OPERATIONS

  /// Assigns the image buffer - use with precaution:
  /// this does not destroy the previous buffer,
  /// it should be explicitly released.
  /// On destruction the buffer IplImage is released.
  /// @param image Preallocated image buffer
  void image(IplImage * image);

  /// Obtains the most recent image, writes its part corresponding
  /// to the provided ROI to the buffer_image.
  /// Receives the time of the request in time parameter
  /// and fills it with the current time, if necessary.
  /// @param image Image to write results to
  /// @param roi Region of interest of a source image to be copied into the buffer
  /// @param time Request timestamp, to be filled with current image timestamp
  virtual void recompute_image(IplImage* buffer_image,
                               const ip_RoiWindow& roi,
                               boost::posix_time::ptime& time) = 0;

  /// When the image size is not known from the beginning (ROS, image
  /// dir, video file, as opppsed to Webcam), this function allocation
  /// mImage
  virtual void allocate_buffer() {}

protected:
  /// Size of original media (image, video, webcam, etc.) useful when
  /// saving results for evaluation. The process can be done at a
  /// lower resolution, but should be saved in original size, compared
  /// to ground truth
  int mOriginalWidth;
  int mOriginalHeight;

  IplImage *mImage; // image to store results to
  boost::posix_time::ptime mTime; // timestamp indicating when the image was acquired
  bool mIsValid;  // current results are valid ?
  boost::shared_mutex m_ImageRequestedMutex;
  uint64_t mDataTime; // Time of data (when processed offline or with ROS), usually in nanosecond

};

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDER_H__
