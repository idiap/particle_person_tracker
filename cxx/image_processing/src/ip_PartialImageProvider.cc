// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_PartialImageProvider - provides images that are part of some calculation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_PartialImageProvider.h>    // declaration of this

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_PartialImageProvider::ip_PartialImageProvider(
        ip_ImageProvider * master_provider, IplImage * buffer_image) :
        mMasterProvider(master_provider) {
        image(buffer_image);
}

/* virtual */
ip_PartialImageProvider::~ip_PartialImageProvider() {
    // override default behaviour, do not release the image
    image(0);
}

/* virtual */ void
ip_PartialImageProvider::recompute_image(IplImage* image,
        const ip_RoiWindow& roi, boost::posix_time::ptime& time) {
       // forces recomputation if not yet up-to-date
       mMasterProvider->image(roi);
       time = mMasterProvider->time();
}

} // namespace ImageProcessing
