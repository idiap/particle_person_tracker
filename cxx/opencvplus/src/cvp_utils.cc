/**
 * @file cxx/opencvplus/src/cvp_utils.cc
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

// LOCAL INCLUDES
#include "opencvplus/cvp_utils.h"

using namespace std;

namespace OpenCvPlus {

bool intersect(const CvRect& roi1, const CvRect& roi2, CvRect& roi_result) {
    // evaluate Y-coord and height
    if (roi1.y >= roi2.y) {
        if (roi1.y >= roi2.y + roi2.height) {
            return false;
        } else {
            roi_result.y = roi1.y;
        }
    } else {
        if (roi1.y + roi1.height <= roi2.y) {
            return false;
        } else {
            roi_result.y = roi2.y;
        }
    }
    int last_row = min(roi1.y + roi1.height, roi2.y + roi2.height);
    roi_result.height = last_row - roi_result.y;

    // evaluate X-coord and width
    if (roi1.x >= roi2.x) {
        if (roi1.x >= roi2.x + roi2.width) {
            return false;
        } else {
            roi_result.x = roi1.x;
        }
    } else {
        if (roi1.x + roi1.width <= roi2.x) {
            return false;
        } else {
            roi_result.x = roi2.x;
        }
    }
    int last_col = min(roi1.x + roi1.width, roi2.x + roi2.width);
    roi_result.width = last_col - roi_result.x;

    return (roi_result.height > 0) && (roi_result.width > 0);

} // intersect

} // namespace OpenCvPlus
