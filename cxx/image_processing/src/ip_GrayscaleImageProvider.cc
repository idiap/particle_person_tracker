// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_GrayscaleImageProvider - transforms input images to grayscale
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_GrayscaleImageProvider.h>  // declaration of this

using namespace std;

namespace ImageProcessing {


ip_GrayscaleImageProvider::
ip_GrayscaleImageProvider(ip_ImageProvider * img_provider):
  mSourceImageProvider(img_provider)
{
  int width =  img_provider->image_buffer()->width;
  int height = img_provider->image_buffer()->height;
  int depth =  img_provider->image_buffer()->depth;

  // create the grayscale image using the source image depth

  IplImage * grayscale_image = cvCreateImage(cvSize(width, height), depth, 1);
  image(grayscale_image);
}


void
ip_GrayscaleImageProvider::
recompute_image(IplImage* buffer_image,
                const ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  IplImage* source_image = mSourceImageProvider->image();
  time = mSourceImageProvider->time();
  // assume the source image is read by opencv
  // the source colour space is then BGR
  cvCvtColor(source_image, buffer_image, CV_BGR2GRAY);
} // recompute_image


} // namespace ImageProcessing
