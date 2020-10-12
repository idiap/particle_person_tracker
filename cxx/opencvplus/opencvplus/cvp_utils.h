/**
 * @file cxx/opencvplus/opencvplus/cvp_utils.h
 * @date 06 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Various utility functions for OpenCV.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_UTILS_H__
#define __CVP_UTILS_H__

// SYSTEM INCLUDES
#include <cv.h>

namespace OpenCvPlus {

/// Intersects two rectangular regions
/// @param roi1 First region
/// @param roi2 Second region
/// @param roi_result Stores intersection of the two rectangular regions;
/// contents in not defined if the two provided regions do not intersect
/// @return True if the two regions intersect, False otherwise
bool intersect(const CvRect& roi1, const CvRect& roi2, CvRect& roi_result);

} // namespace OpenCvPlus

#endif // __CVP_UTILS_H__
