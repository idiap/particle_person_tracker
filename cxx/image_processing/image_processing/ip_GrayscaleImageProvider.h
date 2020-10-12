// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_GrayscaleImageProvider - transforms input images to grayscale
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_GRAYSCALEIMAGEPROVIDER_H__
#define __IP_GRAYSCALEIMAGEPROVIDER_H__

// LOCAL INCLUDES
#include "ip_ImageProvider.h"        // base class

namespace ImageProcessing {

/// @brief Class to transform input images to grayscale
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_GrayscaleImageProvider : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor to initialize grayscale image provider
    /// @param img_provider Input image provider
    ip_GrayscaleImageProvider(ip_ImageProvider * img_provider);

    // OPERATIONS
  virtual std::string name() { return "[ip_GrayscaleImageProvider]"; }
    /// Overrides base class method, returns IP_IMG_PROVIDER_GRAYSCALE
    /// @return Provider's ID = IP_IMG_PROVIDER_GRAYSCALE
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GRAYSCALE;
    };

    protected:

    // OPERATIONS

    /// Overrides base class method, obtains the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    ip_ImageProvider * mSourceImageProvider;     // source image provider

};

} // namespace ImageProcessing

#endif // __IP_GRAYSCALEIMAGEPROVIDER_H__
