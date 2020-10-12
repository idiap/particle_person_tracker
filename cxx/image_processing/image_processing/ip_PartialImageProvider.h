// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_PartialImageProvider - provides images that are part of some calculation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_PARTIALIMAGEPROVIDER_H__
#define __IP_PARTIALIMAGEPROVIDER_H__

// LOCAL INCLUDES
#include "ip_ImageProvider.h"       // base class

namespace ImageProcessing {

/// @brief Provides images that are produced by some larger calculation
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_PartialImageProvider : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor
    ip_PartialImageProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image);

    virtual ~ip_PartialImageProvider();

    protected:

    // OPERATIONS

    /// Overrides base method, makes master provider compute the most recent
    /// image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    ip_ImageProvider * mMasterProvider;

};

} // namespace ImageProcessing

#endif // __IP_PARTIALIMAGEPROVIDER_H__
