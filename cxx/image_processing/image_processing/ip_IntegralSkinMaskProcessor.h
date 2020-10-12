// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_IntegralSkinMaskProcessor - computes integral skin mask
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_INTEGRALSKINMASKPROCESSOR_H__
#define __IP_INTEGRALSKINMASKPROCESSOR_H__

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>      // for real typedef

// LOCAL INCLUDES
#include "ip_ImageProvider.h"       // base class

namespace ImageProcessing {

/// @brief Class to compute integral skin mask image
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    01.03.2011

class ip_IntegralSkinMaskProcessor : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param skin_mask_provider Provider of a skin mask
    ip_IntegralSkinMaskProcessor(ip_ImageProvider * skin_mask_provider);

    protected:

    // OPERATIONS

    /// Obtain the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // provider of gradient angle images
    ip_ImageProvider * mSkinMaskProvider;

};

} // namespace ImageProcessing

#endif // __IP_INTEGRALSKINMASKPROCESSOR_H__
