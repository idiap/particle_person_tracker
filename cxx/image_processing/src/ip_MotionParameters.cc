/**
 * @file cxx/image_processing/src/ip_MotionParameters.cc
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

// LOCAL INCLUDES
#include <image_processing/ip_MotionParameters.h>      // declaration of this

namespace ImageProcessing {

/* static */ const unsigned ip_MotionParameters::TRANSLATION_X_IDX = 0;

/* static */ const unsigned ip_MotionParameters::TRANSLATION_Y_IDX = 1;

/* static */ const unsigned ip_MotionParameters::DIV_X_IDX = 2;

std::ostream& operator<<(std::ostream& os,
        const ip_MotionParameters& motion_params) {
    return os;
} // operator<<

} // namespace ImageProcessing
