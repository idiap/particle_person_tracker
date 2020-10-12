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

#ifndef __IP_FLIPPEDIMAGEPROCESSOR_H__
#define __IP_FLIPPEDIMAGEPROCESSOR_H__

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                             // base class

namespace ImageProcessing {

/// @brief Class to flip images horizontally and/or vertically
///
/// This class flips images horizontally and/or vertically
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date 24.10.2012

class ip_FlippedImageProcessor : public ip_ImageProvider {

    public:

    /// Definitions of types of flip actions (over X, Y or X and Y axes)
    enum FlipType {FLIP_X, FLIP_Y, FLIP_XY};

    // LIFECYCLE

    /// Constructor
    /// @param img_provider Input image provider
    /// @param flip_axis Flip axis - can be X, Y and XY
    ip_FlippedImageProcessor(ip_ImageProvider * img_provider,
        FlipType flip_axis = FLIP_X);

    /// Overrides base class method, returns IP_IMG_PROVIDER_DATA
    /// @return Provider's ID = IP_IMG_PROVIDER_DATA
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType
    id() const {
        return IP_IMG_PROVIDER_DATA;
    };

    protected:

    // OPERATIONS

    /// Overrides base class method, obtains the most recent image
    /// @param image Image to write results to
    virtual void
    recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // provider of source images
    ip_ImageProvider * mSourceImageProvider;
    // opencv encoding for flip direction
    int m_OpenCvFlipMode;

};

} // namespace ImageProcessing

#endif // __IP_FLIPPEDIMAGEPROCESSOR_H__
