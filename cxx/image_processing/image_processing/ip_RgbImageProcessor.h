// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_RgbImageProcessor - class to extract colour channels from images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_RGBIMAGEPROCESSOR_H__
#define __IP_RGBIMAGEPROCESSOR_H__

// SYSTEM INCLUDES
#include <cv.h>                     // IplImage from OpenCV

// LOCAL INCLUDES
#include "ip_ImageProviderGroup.h"  // base class

namespace ImageProcessing {

/// @brief Class to extract colour channels from images
///
/// This class extracts red, green and blue channels of an image into three
/// separate images
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_RgbImageProcessor : public ip_ImageProviderGroup {

    public:

    // LIFECYCLE

    /// Constructor
    ip_RgbImageProcessor(ip_ImageProvider * src_img_provider);

    /// Destructor, deallocates partial image providers
    virtual ~ip_RgbImageProcessor();

    // OPERATIONS

    /// Provider for the red channel of the source image
    /// @return Provider for the red channel of the source image
    ip_ImageProvider* getRedChannelProvider();

    /// Provider for the green channel of the source image
    /// @return Provider for the green channel of the source image
    ip_ImageProvider* getGreenChannelProvider();

    /// Provider for the blue channel of the source image
    /// @return Provider for the blue channel of the source image
    ip_ImageProvider* getBlueChannelProvider();

    protected:

    // OPERATIONS

    /// Obtain the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi);

    private:

    // provider of source images
    ip_ImageProvider * mSourceImageProvider;

    ip_ImageProvider * mRedChannelProvider;
    ip_ImageProvider * mGreenChannelProvider;
    ip_ImageProvider * mBlueChannelProvider;

};

} // namespace ImageProcessing

#endif // __IP_RGBIMAGEPROCESSOR_H__
