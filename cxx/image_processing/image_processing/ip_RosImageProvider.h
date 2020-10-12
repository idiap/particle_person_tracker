/**
 * @file cxx/image_processing/src/ip_RosImageProvider.h
 * @date 21 April 2015
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Adapts images received from ROS to ImageProvider interface
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_ROSIMAGEPROVIDER_H__
#define __IP_ROSIMAGEPROVIDER_H__

#ifndef __ROSINTEGRATION_FOUND__
#error ROS integration was not found, ip_RosImageProvider should not be used!
#endif


// SYSTEM INCLUDES
#include <boost/circular_buffer.hpp>            // boost circular buffer
#include <boost/shared_ptr.hpp>                 // boost shared ptr
#include <boost/thread/mutex.hpp>               // boost mutex
#include <image_transport/image_transport.h>    // for ROS image transport
#include <sensor_msgs/Image.h>                  // for ROS image message
#include <time.h>                               // for ROS time

#if defined(EXIT)
#undef EXIT
#endif

// LOCAL INCLUDES
#include "image_processing/ip_ImageProvider.h"  // image provider

namespace ImageProcessing {

/**
 *  This class serves as a proxy between ROS and the TTrack modules.
 *  ROS provides images that can be converted to OpenCV IplImages.
 *  These are passed to an ip_RosImageProvider instance to which all the
 *  TTrack providers are typically subscribed.
 */

class ip_RosImageProvider: public ip_ImageProvider
{

public:

  // LIFECYCLE

  /// Constructor
  /// @param scope_address Address of the ROS image provider
  /// @param width Preferred image width
  /// @param height Preferred image height
  ip_RosImageProvider(const std::string& scope_address,
                      int width, int height,
                      double scale = 1.0);

  /// Destructor
  virtual ~ip_RosImageProvider();

  virtual std::string name() { return "[ip_RosImageProvider]"; }
  // OPERATIONS

  /// Overrides base class method, returns IP_IMG_PROVIDER_ROS
  /// @return Provider's ID: IP_IMG_PROVIDER_ROS
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const {
    return m_Type;
  };

  /// Overrides base class method.
  /// Returns current image ID as given by ROS event sequence number.
  /// The ID is equal to image 0-based number in the
  /// ROS visual stream.
  /// @return Current image ID.
  virtual unsigned image_id() const;

  /// Overrides base class method.
  /// Returns calculated FPS (frames per second) rate.
  /// Returns -1 if the FPS rate cannot be evaluated (e.g., for online
  /// camera capture when capturing has not yet been started).
  /// @return Image provider FPS (frames per second) rate.
  virtual float fps() const;

  /// Overrides base class method.
  /// Returns sensor_msgs::ImageConstPtr.header.frame_id
  virtual std::string get_tf_frame();

  virtual uint64_t get_data_time_ns() { return m_RosTime.toNSec(); }

  virtual bool should_wait() const { return true; }

protected:

  virtual void recompute_image(IplImage* buffer_img,
                               const ip_RoiWindow& roi,
                               boost::posix_time::ptime& time);

  virtual void allocate_buffer();

private:

  // Callback function called when an image is received from ROS
  void image_received(const sensor_msgs::ImageConstPtr& msg);

  // main ROS node handle
  ros::NodeHandle m_RootNH;
  // ROS image transport
  image_transport::ImageTransport m_ImageTransport;
  // ROS image subscriber
  image_transport::Subscriber m_ImageSubscriber;

  double m_Scale;
  int m_ImageTopicHeight;
  int m_ImageTopicWidth;

  // mutex to protect read/write of buffer image
  boost::mutex m_ImageMutex;
  // pointer to ROS image
  sensor_msgs::ImageConstPtr m_RosImagePtr;
  // buffer to store ROS time
  ros::Time m_RosTime;

  // event that was received as an input
  boost::uint32_t m_CurrentEvendSeqNumber;

  boost::circular_buffer<float> m_FpsValues; // delay-based fps values
  bool m_FirstFrame;                    // current frame is the first one?
  boost::posix_time::ptime m_LastImageTimeStamp;

  ip_ImageProviderType m_Type;

  int m_TotalMsgReceived;

}; // RosImageProvider

} // namespace ImageProcessing

#endif // __IP_ROSIMAGEPROVIDER_H__
