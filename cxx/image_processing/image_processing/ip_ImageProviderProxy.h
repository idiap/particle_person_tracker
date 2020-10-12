// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderProxy - proxy image provider
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_IMAGEPROVIDERPROXY_H__
#define __IP_IMAGEPROVIDERPROXY_H__

// LOCAL INCLUDES
#include "ip_ImageProvider.h"

namespace ImageProcessing {

/// @brief Class for a proxy image provider
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_ImageProviderProxy : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor to initialize proxy provider
    /// @param img_provider Input image provider
    ip_ImageProviderProxy(ip_ImageProvider * img_provider);

    /// Destructor, deallocates image buffer
    virtual ~ip_ImageProviderProxy();

    protected:

    // OPERATIONS

    /// Obtain the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    ip_ImageProvider * mSourceImageProvider;     // source image provider
    IplImage*  mImage;                           // image to store results to

};

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDERPROXY_H__
