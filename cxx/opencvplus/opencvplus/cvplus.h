/**
   The OpenCV+ library.

   An extension of OpenCV that is fully dependent on OpenCV for basic
   data types and functionality, and that providesadditional
   functionality for more specialized image processing.

   See COPYING file for the complete license text.
 */

#ifndef __CVPLUS_H__
#define __CVPLUS_H__

#include <cv.h>
#ifndef NOPENCV
  #include <opencv/cxcore.h>
#endif
#include <highgui.h>

namespace OpenCvPlus {
  typedef float real;

} // namespace OpenCvPlus

// Exceptions
#include "cvp_Exceptions.h"

// General purpose functions
#include "cvp_helper_functions.h"

// Histogram-of-gradients features
#include "cvp_gradient_histogram_features.h"

// Skin color detection
#include "cvp_skin_color.h"

// Single frame head pose estimation
#include "cvp_head_pose.h"

#endif
