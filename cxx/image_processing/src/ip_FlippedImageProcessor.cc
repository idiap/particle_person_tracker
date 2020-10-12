/**
 * @file cxx/image_processing/image_processing/ip_FlippedImageProcessor.h
 * @date 24 October 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image processor that flips images horizontally and/or vertically
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// LOCAL INCLUDES
#include <image_processing/ip_FlippedImageProcessor.h>  // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_FlippedImageProcessor::ip_FlippedImageProcessor(
        ip_ImageProvider * img_provider, FlipType flip_axis) :
        mSourceImageProvider(img_provider) {

    switch (flip_axis) {
    case FLIP_X: m_OpenCvFlipMode = 1; break;
    case FLIP_Y: m_OpenCvFlipMode = 0; break;
    case FLIP_XY: m_OpenCvFlipMode = -1; break;
    }

    int width =  img_provider->image_buffer()->width;
    int height = img_provider->image_buffer()->height;
    int depth =  img_provider->image_buffer()->depth;
    int nChannels =  img_provider->image_buffer()->nChannels;

    // create the flipped image using the source image parameters

    IplImage * flipped_image = cvCreateImage(
        cvSize(width, height), depth, nChannels);
    image(flipped_image);

}

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void
ip_FlippedImageProcessor::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {
    IplImage* source_image = mSourceImageProvider->image();
    time = mSourceImageProvider->time();

    // flip input image
    cvFlip(source_image, buffer_image, m_OpenCvFlipMode);
} // recompute_image

} // namespace ImageProcessing
