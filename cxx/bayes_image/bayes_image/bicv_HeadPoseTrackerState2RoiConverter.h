/**
 * @file cxx/bayes_image/bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h
 * @date 15 February 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Converts ROI from / to tracker state
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_HEADPOSETRACKERSTATE2ROICONVERTER_H__
#define __BICV_HEADPOSETRACKERSTATE2ROICONVERTER_H__

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>              // ROI window

// LOCAL INCLUDES
#include "bicv_HeadPoseTrackerState.h"                  // head pose parameters
#include "bicv_GeneralTrackerState.h"                   // tracker parameters

namespace BICV {

/// @brief Converter of ROI to/from tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct HpParams2RoiConverter {

    /// Converts a region of interest represented by a bounding box into
    /// head tracker parameters represented by offset, scale and excentricity
    /// @param roi Bounding box of the region of interest
    /// @return Head pose tracker parameters
    static bicv_HeadPoseParameters roi2hpparams(
            const ImageProcessing::ip_RoiWindow& roi);

    /// Converts head tracker parameters represented by offset, scale and
    /// excentricity into a region of interest represented by its bounding box
    /// @param hpparams Head pose tracker parameters
    /// @return Bounding box of the region of interest
    static ImageProcessing::ip_RoiWindow hpparams2roi(
            const bicv_HeadPoseParameters& hpparams);

    static bicv_TrackerParameters roi2params(
            const ImageProcessing::ip_RoiWindow& roi);
    static ImageProcessing::ip_RoiWindow params2roi(
            const bicv_TrackerParameters& params);

};

}

#endif // __BICV_HEADPOSETRACKERSTATE2ROICONVERTER_H__
