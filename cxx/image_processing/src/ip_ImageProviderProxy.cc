// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderProxy - proxy image provider
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <iostream>

// LOCAL INCLUDES
#include <image_processing/ip_ImageProviderProxy.h>  // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_ImageProviderProxy::ip_ImageProviderProxy(ip_ImageProvider * img_provider) :
        mSourceImageProvider(img_provider), mImage(0) {

    int width =  img_provider->image_buffer()->width;
    int height = img_provider->image_buffer()->height;
    int depth =  img_provider->image_buffer()->depth;
    int nchannels =  img_provider->image_buffer()->nChannels;

    // create the grayscale image using the source image depth

    IplImage * buffer_image = cvCreateImage(cvSize(width, height), depth,
            nchannels);
    image(buffer_image);

} // ip_ImageProvider

/* virtual */ ip_ImageProviderProxy::~ip_ImageProviderProxy() {
    if (mImage) {
        cvReleaseImage(&mImage);
        mImage = 0;
    }
} // ~ip_ImageProvider

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_ImageProviderProxy::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

//    cout << "ImageProviderProxy, current time=" << time << endl;

    // thread-safe image copy
    mSourceImageProvider->copy_image(buffer_image, time);
} // recompute_image

} // namespace ImageProcessing
