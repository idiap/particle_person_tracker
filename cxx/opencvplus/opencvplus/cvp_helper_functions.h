/**
 * @file cxx/opencvplus/opencvplus/cvp_helper_functions.h
 * @date 01 June 2012
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Helper functions for the OpenCV+ library. A collection of unrelated
 *        functions with simple functionality that are used througout the
 *        library.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_HELPER_FUNCTIONS_H__
#define __CVP_HELPER_FUNCTIONS_H__

#include "cvplus.h"

namespace OpenCvPlus {

    /// scaled arctangent function, returns 100 * arctan(x)
    /// NOTE: this (arctan) is function defined in utils.cc;
    /// should be faster than math.h atan function
    double cvp_arctan(double x);

    /// Returns a pixel value from an 8-bit, single channel OpenCV image.
    /// @param ipImage The source image.
    /// @param iRow The y-coordinate of the pixel.
    /// @param iColumn The x-coordinate of the pixel.
    inline uchar cvp_PixVal8U(IplImage const* ipImage, int iRow, int iColumn) {
        return ((uchar*)(ipImage->imageData + iRow*ipImage->widthStep))[iColumn];
    }

    /// Normalize a real vector so that it's 2-norm is 1.
    /// @param iopVector The vector to normalize in place.
    /// @param iVectorLength The number of entries in the vector.
    void cvp_NormalizeRealVector(real* iopVector, int iVectorLength);

    /// Convert from BGR to YPbPr color space.
    /// BGR values should be in range [0,255]. The output image has
    /// the same shape as the input with YPbPr values in range [0,1].
    /// @param ipImage Input BGR image (3 channels, IPL_DEPTH_8U)
    /// @param opImage Output YPbPr image (3 channels, IPL_DEPTH_32F)
    /// @param roi Region of interest for which to convert the data
    void bgr2ypbpr(const IplImage* ipImage, IplImage* opImage,
            const CvRect& roi);

    /// Convert pixel value from BGR to YPbPr color space.
    /// BGR values in each channel should be in range [0,255]. The output is a
    /// YPbPr value with channel components in [0,1].
    /// @param bgr_val Input BGR pixel
    /// @return Output YPbPr pixel
    CvScalar bgr2ypbpr(const CvScalar& bgr_val);

    /// Convert pixel value from YPbPr to BGR color space.
    /// YPbPr values in each channel should be in range [0,1]. The output is a
    /// BGR value with channel components in [0,255].
    /// @param bgr_val Input YPbPr pixel
    /// @return Output BGR pixel
    CvScalar ypbpr2bgr(const CvScalar& ypbpr_val);

} // namespace OpenCvPlus

#endif
