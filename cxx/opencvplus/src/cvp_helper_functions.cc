/**
 * @file cxx/opencvplus/src/cvp_helper_functions.cc
 * @date 01 June 2012
   @author Carl Scheffler (Carl.Scheffler@idiap.ch)
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

// LOCAL INCLUDES
#include <opencvplus/cvp_helper_functions.h>
#include <opencvplus/cvp_IplDepthTraits.h>

static const double HALF_PI = 1.570796325;
static const int SCALE = 100;

namespace OpenCvPlus {

  double cvp_arctan(double x) {
    int sign = SCALE;

    if(x < 0.) {
        x = -x;
        sign = -SCALE;
    }
    if(x <= 1) {
        x /= 1 + x * x * 0.28;
    } else {
        x = HALF_PI - x / (x * x + 0.28);
    }
    return sign * x;
  }

  void cvp_NormalizeRealVector(real* iopVector, int iVectorLength) {
    real norm = 0;
    for(int i = 0; i < iVectorLength; i++)
      norm += iopVector[i]*iopVector[i];
    norm = sqrt(norm);
    for(int i = 0; i < iVectorLength; i++)
      iopVector[i] /= norm;
  }

  void bgr2ypbpr(const IplImage* ipImage, IplImage* opImage, const CvRect& roi) {

  //    static fcm_real const A[] = { 0.299/255,     0.587/255,     0.114/255,
  //          -0.168736/255, -0.331264/255,  0.5/255,
  //           0.5/255,      -0.418688/255, -0.081312/255};
  //    static fcm_real const B[] = {0, 0.5, 0.5};

      // verify input and output image formats (DEBUG mode only)
      assert((ipImage->depth == IPL_DEPTH_8U) && (ipImage->nChannels == 3));
      assert((opImage->depth == IPL_DEPTH_32F) && (opImage->nChannels == 3));
      assert((opImage->width == opImage->width) &&
             (ipImage->height == opImage->height));

      const int start_row = roi.y;
      const int end_row = roi.y + roi.height;
      const int start_col= roi.x;
      const int end_col = roi.x + roi.width;

      typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32F>::type float_32;

      // alignment correction
      const unsigned char * pImageData = (unsigned char *)ipImage->imageData;
      unsigned char * pOutput = (unsigned char *)(opImage->imageData);

      const unsigned char * pInputCurrentStart = pImageData +
              roi.y * ipImage->widthStep + roi.x * 3;
      unsigned char * pOutputCurrentStart = pOutput +
              roi.y * opImage->widthStep + roi.x * 3 * 4;

      for(int row = start_row; row < end_row ; row++) {
          const unsigned char * pInputCurrent = pInputCurrentStart;
          float_32 * pOutputCurrent = (float_32 *) pOutputCurrentStart;
          for(int col = start_col; col < end_col; col++) {
              const unsigned char b = *pInputCurrent++;
              const unsigned char g = *pInputCurrent++;
              const unsigned char r = *pInputCurrent++;
              *pOutputCurrent++ = (r * 0.299 + g * 0.587 + b * 0.114) / 255;
              *pOutputCurrent++ = (r * (-0.168736) + g * (-0.331264) + b * 0.5) / 255
                  + 0.5;
              *pOutputCurrent++ = (r * 0.5 + g * (-0.418688) + b * (-0.081312)) / 255
                    + 0.5;
          //            const int b = *pImageData++;
          //            const int g = *pImageData++;
          //            const int r = *pImageData++;
          //            *pOutput++ = static_cast<float>(r * 299 + g * 587 + b * 114) / 255000;
          //            *pOutput++ = static_cast<float>(r * (-169) + g * (-331) + b * 500 + 127500) / 255000;
          //            *pOutput++ = static_cast<float>(r * 500 + g * (-419) + b * (-81) + 127500) / 255000;
          }
          // move to the next row in the input and output images
          pInputCurrentStart += ipImage->widthStep;
          pOutputCurrentStart += opImage->widthStep;
      }
  } // bgr2ypbpr

  CvScalar bgr2ypbpr(const CvScalar& bgr_val) {

      CvScalar ypbpr_val;
      const double * pInputCurrent = bgr_val.val;
      double * pOutputCurrent = ypbpr_val.val;

      const double b = *pInputCurrent++;
      const double g = *pInputCurrent++;
      const double r = *pInputCurrent++;

      *pOutputCurrent++ = (r * 0.299 + g * 0.587 + b * 0.114) / 255;
      *pOutputCurrent++ = (r * (-0.168736) + g * (-0.331264) + b * 0.5) / 255
          + 0.5;
      *pOutputCurrent++ = (r * 0.5 + g * (-0.418688) + b * (-0.081312)) / 255
            + 0.5;
      return ypbpr_val;
  } // bgr2ypbpr

  CvScalar ypbpr2bgr(const CvScalar& ypbpr_val) {
      CvScalar bgr_val;
      const double * pInputCurrent = ypbpr_val.val;
      double * pOutputCurrent = bgr_val.val;

      const double y = *pInputCurrent++;
      const double pb = *pInputCurrent++ - 0.5;
      const double pr = *pInputCurrent++ - 0.5;

      // b
      *pOutputCurrent++ = (y + pb * (1.772000066073816) + pr * ( 0.000000406298063)) * 255;
      // g
      *pOutputCurrent++ = (y + pb * (-0.344135678165337) + pr * (-0.714136155581812)) * 255;
      // r
      *pOutputCurrent++ = (y + pb * (-0.000001218894189) + pr * (1.401999588657340)) * 255;
      return bgr_val;
  } // ypbpr2bgr

} // namespace OpenCvPlus
