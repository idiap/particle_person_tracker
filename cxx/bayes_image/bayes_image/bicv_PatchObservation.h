// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_PatchObservation - class for an image patch observation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_PATCHOBSERVATION_H__
#define __BICV_PATCHOBSERVATION_H__

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>          // ROI window

// SYSTEM INCLUDES
#include <cv.h>                                     // for IplImage from OpenCV

namespace BICV {

/// @brief Class for an image patch observation
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct bicv_PatchObservation {
    bicv_PatchObservation(
            IplImage* image,
            const ImageProcessing::ip_RoiWindow& patch) :
            m_Image(image), m_Roi(patch) {
    }
    IplImage* m_Image;
    const ImageProcessing::ip_RoiWindow m_Roi;
};

} // namespace BICV

#endif // __BICV_PATCHOBSERVATION_H__
