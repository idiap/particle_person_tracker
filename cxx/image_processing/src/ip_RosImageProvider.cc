/**
 * @file cxx/image_processing/src/ip_RosImageProvider.cc
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

// SYSTEM INCLUDES
#include <iostream>
#include <numeric>                                   // accumulate
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>             // for image encodings

// LOCAL INCLUDES
#include <image_processing/ip_RosImageProvider.h>    // declaration of this
#include <image_processing/ip_Exceptions.h>          // for ip_Exception

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// buffer size to keep arriving RSB events
static const int FPS_VALUES_MAXCOUNT = 10;

using namespace std;

namespace ImageProcessing {

ip_RosImageProvider::
ip_RosImageProvider(const std::string& topic_name,
                    int width,
                    int height,
                    double scale):
  m_RootNH("ros_img_provider"),
  m_ImageTransport(m_RootNH),
  m_Scale(scale),
  m_ImageTopicHeight(0),
  m_ImageTopicWidth(0),
  m_CurrentEvendSeqNumber(0),
  m_FpsValues(FPS_VALUES_MAXCOUNT),
  m_Type(IP_IMG_PROVIDER_ROS)
{
  // Check if we should subscribe to the compressed topic by checking
  // whether the topic name is
  //
  // /topic/name/image_raw/compressed
  const std::string suffix = "/compressed";
  std::string transport = "raw";
  std::string new_topic_name(topic_name);
  if(suffix.size() <= topic_name.size())
    {
      if(std::equal(suffix.rbegin(), suffix.rend(),
                    topic_name.rbegin()))
        {
          transport = "compressed";
          const size_t topic_length = topic_name.size();
          const size_t suffix_length = suffix.size();
          new_topic_name = topic_name.substr(0, topic_length - suffix_length);
        }
    }

  m_RootNH.param("image_transport", transport, std::string(transport));
  image_transport::TransportHints hints(transport,
                                        ros::TransportHints(),
                                        m_RootNH);

  std::cout << name() << " Subscribing to " << new_topic_name
            << " with transport '" << transport << "'" << std::endl;

  m_ImageSubscriber =
    m_ImageTransport.subscribe(new_topic_name, 1,
                               &ip_RosImageProvider::image_received,
                               this, hints);

  // IplImage *img8c3 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  // cvZero(img8c3);
  // image(img8c3);
  // cvReleaseImage(&img8c3);
  // mImage = 0;
  m_TotalMsgReceived = 0;
  m_FirstFrame = true;
}


ip_RosImageProvider::
~ip_RosImageProvider()
{
}


unsigned
ip_RosImageProvider::
image_id() const
{
  return m_CurrentEvendSeqNumber;
}


float
ip_RosImageProvider::fps() const
{
  if(m_FpsValues.empty())
    {
      return 0;
    }
  else
    {
      return accumulate(m_FpsValues.begin(), m_FpsValues.end(), 0.0) /
        m_FpsValues.size();
    }
}


void
ip_RosImageProvider::
allocate_buffer()
{
  if(m_ImageTopicWidth > 0 && m_ImageTopicHeight > 0)
    {
      std::cout << name() << " Image topic of size "
                << m_ImageTopicWidth << "x" << m_ImageTopicHeight
                << std::endl;

      int H1 = static_cast<int>(m_Scale*m_ImageTopicHeight);
      int W1 = static_cast<int>(m_Scale*m_ImageTopicWidth);

      std::cout << name() << " Allocate size "
                << W1 << "x" << H1 << std::endl;

      IplImage *img8c3 = cvCreateImage(cvSize(W1, H1), IPL_DEPTH_8U, 3);
      cvZero(img8c3);
      image(img8c3);
    }
}


void
ip_RosImageProvider::
recompute_image(IplImage *buf_img,
                const ImageProcessing::ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  if (m_RosImagePtr)
    {
      // /////////////////////////////////////////////////////////////////
      // // HWP modif for color space
      // IplImage img_header;
      // cvInitImageHeader(&img_header,
      //     cvSize(m_RosImagePtr->width, m_RosImagePtr->height),
      //     IPL_DEPTH_8U, 3);
      // // VK: ugly const_cast required!
      // // we need to wrap ROS buffer into an OpenCV image
      // // but SetData method does not work with const pointers to data
      // // (obviously). cvResize does not modify the source image, so
      // // de facto constness is kept
      // cvSetData(&img_header,
      //     const_cast<unsigned char*>(&m_RosImagePtr->data[0]),
      //     m_RosImagePtr->step);

      // cvResize(&img_header, buf_img, CV_INTER_LINEAR);

      // /////////////////////////////////////////////////////////////////
      // // FIX ME: cv_bridge and opencv2/3 breaks seg fault. It
      // // would be better to use cv_bridge::toCvCopy() but it
      // // crashes. OpenCV 3 only should be used but it might
      // // involve installing lots of packags and recompiling from
      // // source
      // cv::Mat cp1 = cv::cvarrToMat(buf_img, false);
      // cv::MatIterator_<cv::Vec3b> it, end;
      // for(it=cp1.begin<cv::Vec3b>() ; it != cp1.end<cv::Vec3b>(); ++it)
      //   {
      //     unsigned char r = (*it)[2];
      //     unsigned char b = (*it)[0];
      //     (*it)[0] = r;
      //     (*it)[2] = b;
      //   }
      // /////////////////////////////////////////////////////////////////

      /////////////////////////////////////////////////////////////////
      // convert ros image to cv image by cv_bridge
      // modified by hwp
      cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(m_RosImagePtr, "bgr8"); // ROS -> Mat

      IplImage img(cv_ptr->image);  // Mat -> IplImage
      cvResize(&img, buf_img, CV_INTER_LINEAR);      // resize
      /////////////////////////////////////////////////////////////////
      time = m_RosTime.toBoost();

      // update FPS data
      if(!m_FirstFrame)
        {
          boost::posix_time::time_duration update_time = time -
            m_LastImageTimeStamp;
          long update_millis = update_time.total_milliseconds();
          if (update_millis <= 0)
            {
              update_millis = 1;
            }
          m_FpsValues.push_back(1000.0 / update_millis);
        }

      m_LastImageTimeStamp = time;
      m_FirstFrame = false;
    }
}


void
ip_RosImageProvider::
image_received(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(m_ImageMutex);
  m_TotalMsgReceived++;
  m_RosImagePtr = msg;
  m_RosTime = msg->header.stamp;
  mDataTime = msg->header.stamp.toNSec();
  m_CurrentEvendSeqNumber = msg->header.seq;

  if(m_ImageTopicWidth == 0)
    {
      mOriginalWidth = msg->width;
      mOriginalHeight = msg->height;
      m_ImageTopicWidth = msg->width;
      m_ImageTopicHeight = msg->height;
    }

  // try
  //   {
  //     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  //     cv::waitKey(30);
  //   }
  // catch (cv_bridge::Exception& e)
  //   {
  //     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  //   }

} // image_received


std::string
ip_RosImageProvider::
get_tf_frame()
{
  if(m_RosImagePtr)
    return m_RosImagePtr->header.frame_id;
  else
    return ip_ImageProvider::get_tf_frame();
}

} // namespace ImageProcessing
