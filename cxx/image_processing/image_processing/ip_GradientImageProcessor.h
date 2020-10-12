/**
 * @file cxx/image_processing/image_processing/ip_GradientImageProcessor.h
 * @date 03 February 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Provides image gradients for each pixel of the original image.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_GRADIENTIMAGEPROCESSOR_H__
#define __IP_GRADIENTIMAGEPROCESSOR_H__

// LOCAL INCLUDES
#include "ip_ImageProviderGroup.h"  // base class

namespace ImageProcessing {

/// @brief Class to extract image gradient for each pixel of the original image
///
/// This class extracts X, Y components, gradient amplitude and angle for each
/// pixel of the original image.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_GradientImageProcessor: public ip_ImageProviderGroup
{
public:

    // LIFECYCLE
  virtual std::string name() { return "[ip_GradientImageProcessor]"; }
    /// Constructor
    /// @param gray_img_provider Grayscale input image
    ip_GradientImageProcessor(ip_ImageProvider * gray_img_provider);

    /// Destructor, deallocates partial image providers
    virtual ~ip_GradientImageProcessor();

    // OPERATIONS

    /// Provider for the X gradient component of the source image
    /// @return Provider for the X gradient component of the source image
    ip_ImageProvider* getGradientXProvider();

    /// Provider for the Y gradient component of the source image
    /// @return Provider for the Y gradient component of the source image
    ip_ImageProvider* getGradientYProvider();

    /// Provider for the gradient magnitude of the source image
    /// @return Provider for the gradient magnitude of the source image
    ip_ImageProvider* getGradientMagnitudeProvider();

    /// Provider for the gradient angle of the source image
    /// @return Provider for the gradient angle of the source image
    ip_ImageProvider* getGradientAngleProvider();

    /// Compute gradient x-component, y-component, magnitude and orientation
    /// for every pixel of an image.
    /// @param ipImage_32F1C Floating point input image.
    /// @param opGradientX_32F1C Output float image: gradient x-component.
    /// @param opGradientY_32F1C Output float image: gradient y-component.
    /// @param opGradientMagnitude_32F1C Output float image: gradient magnitude.
    /// @param opGradientAngle_32F1C Output float image: gradient angle.
    /// @param iMagnitudeCap Gradient magnitude is capped at this value.
    static void compute_gradients(const IplImage * ipImage_32F1C,
        IplImage* opGradientX_32F1C, IplImage* opGradientY_32F1C,
        IplImage* opGradientMagnitude_32F1C, IplImage* opGradientAngle_32F1C,
        float iMagnitudeCap);

protected:

    // OPERATIONS

    /// Obtain the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

private:

    // provider of source images
    ip_ImageProvider * mSourceImageProvider;

    ip_ImageProvider * mGradientXProvider;
    ip_ImageProvider * mGradientYProvider;
    ip_ImageProvider * mGradientMagnitudeProvider;
    ip_ImageProvider * mGradientAngleProvider;

};

} // namespace ImageProcessing

#endif // __IP_GRADIENTIMAGEPROCESSOR_H__
