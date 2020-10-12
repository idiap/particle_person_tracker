// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProvider - class to represent an abstract image provider
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#include <image_processing/ip_ImageProvider.h>


namespace ImageProcessing {


ip_ImageProvider::
ip_ImageProvider():
  mOriginalWidth(-1),
  mOriginalHeight(-1),
  mImage(0),
  mTime(),
  mIsValid(false)
{
}


ip_ImageProvider::
~ip_ImageProvider()
{
  if(mImage)
    {
      cvReleaseImage(&mImage);
      mImage = 0;
    }
}


IplImage*
ip_ImageProvider::
image(const ip_RoiWindow& iroi)
{
  boost::unique_lock<boost::shared_mutex> lock(m_ImageRequestedMutex);

  if(!mIsValid)
    {
      // Dirty hack for ROS, file, and video provider to handle "scale
      // parameter"
      if(!mImage) allocate_buffer();

      ip_RoiWindow roi;
      roi.m_iFirstColumn = iroi.m_iFirstColumn;
      roi.m_iFirstRow = iroi.m_iFirstRow;
      roi.m_iWidth = iroi.m_iWidth;
      roi.m_iHeight = iroi.m_iHeight;

      if(roi.m_iFirstColumn == 0 && roi.m_iFirstRow == 0 &&
         roi.m_iWidth == 0       && roi.m_iHeight == 0 &&
         mImage)
        {
          roi.m_iWidth = mImage->width;
          roi.m_iHeight = mImage->height;
        }

      mTime = boost::posix_time::microsec_clock::local_time();
      recompute_image(mImage, roi, mTime);
      mIsValid = true;
    }

  return mImage;
}


unsigned
ip_ImageProvider::
image_id() const
{
  return 0;
}


void
ip_ImageProvider::
copy_image(IplImage* dest_image,
           boost::posix_time::ptime& time)
{
  boost::unique_lock<boost::shared_mutex> lock(m_ImageRequestedMutex);

  try {
    cvCopy(mImage, dest_image);
  } catch (...) {
    std::cout << "Exception caught in image provider, src_depth=" << mImage->depth <<
      ", src_size=(" << mImage->height << "," << mImage->width <<
      "), dst_depth=" << dest_image->depth <<
      ", dst_size=(" << dest_image->height << "," << dest_image->width <<
      ")" << std::endl;
  }

  time = mTime;
}


boost::posix_time::ptime
ip_ImageProvider::
time() const
{
  return mTime;
}


float
ip_ImageProvider::fps() const
{
    return -1;
}


IplImage*
ip_ImageProvider::image_buffer() const
{
  return mImage;
}


void
ip_ImageProvider::
invalidate()
{
  boost::unique_lock<boost::shared_mutex> lock(m_ImageRequestedMutex);
  mIsValid = false;
}


void
ip_ImageProvider::
image(IplImage *image)
{
  mImage = image;
}


} // namespace ImageProcessing
