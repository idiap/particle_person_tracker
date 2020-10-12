/**
 * @file cxx/image_processing/image_processing/ip_MotionParameters.h
 * @date 15 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Motion parameters representation
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_MOTIONPARAMETERS_H__
#define __IP_MOTIONPARAMETERS_H__

// SYSTEM INCLUDES
#include <vector>                                   // STL vector

// LOCAL INCLUDES
#include <image_processing/ip_RoiWindow.h>          // ROI window

namespace ImageProcessing {

/// @brief Class to represent motion parameters
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 3.0
/// @date    15.03.2013

struct ip_MotionParameters {

    /// index of horizontal motion component
    static const unsigned TRANSLATION_X_IDX /* = 0 */;
    /// index of vertical motion component
    static const unsigned TRANSLATION_Y_IDX /* = 1 */;
    /// index of motion divergence component
    static const unsigned DIV_X_IDX /* = 2 */;

    /// Various motion parameters stored in a vector
    std::vector<double> m_Parameters;
    /// Flags indicating validity of the corresponding motion parameters,
    /// should be of the same size as m_Parameters
    std::vector<bool> m_Flags;
    /// Bounding box for which the parameters were estimated
    ip_RoiWindow m_BoundingBox;

    // LIFECYCLE

    /// Constructs motion parameters of certain size
    ip_MotionParameters(size_t size) : m_Parameters(size),
            m_Flags(size, false) {
    }

}; // struct ip_MotionParameters

/// Stream output operator for motion parameters
std::ostream& operator<<(std::ostream& os,
        const ip_MotionParameters& motion_params);

} // namespace ImageProcessing

#endif // __IP_MOTIONPARAMETERS_H__
