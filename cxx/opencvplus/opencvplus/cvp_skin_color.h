/**
 * Functions for computing skin color models and applying skin color
 * masks. Skin color is modeled as a 2-dimensional Gaussian
 * distribution over red and green pixel values.
 *
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_SKIN_COLOR_H__
#define __CVP_SKIN_COLOR_H__

// SYSTEM INCLUDES
#include <string>

// LOCAL INCLUDES
#include "cvplus.h"
#include "cvp_SkinColourModel.h"

namespace OpenCvPlus {

  /// Compute a skin color mask from a RGB input image using a
  /// Gaussian skin color model.
  /// @param ipInputImage Input RGB image.
  /// @param opOutputImage Storage for skin color mask, of type CV_8UC1.
  /// @param opTempImage1 Temporary storage for computation, of type CV_8UC1.
  /// @param opTempImage2 Temporary storage for computation, of type CV_8UC1.
  /// @param ipSkinColor A length-5 array with [mean green, mean red, variance
  ///                    green, variance red, covariance green-red] statistics.
  /// @param iDetectionThreshold Threshold below which -0.5 times the
  ///                            unnormalised log likelihood of the Gaussian
  ///                            skin model is accepted as indicating skin.
  ///                            TODO: Make this something more sensible.
/// @param skin_mask_value Skin mask value
  void cvp_ComputeSkinMask(IplImage const* ipInputImage, IplImage* opOutputImage,
			   IplImage* opTempImage1, IplImage* opTempImage2,
			   const cvp_SkinColourModel& skin_colour_model,
			   real iDetectionThreshold,
			   real skin_mask_value);

  /// Update an existing skin color model based on an input image.
  /// @param iopSkinColor The existing skin color model. Will be
  ///                     updated in place.
  /// @param ipImage Input image used to update the color model.
  /// @param iY First row of the bounding box within which to search
  ///           for skin color pixels.
  /// @param iX First column of the bounding box within which to
  ///           search for skin color pixels.
  /// @param iHeight Height of bounding box.
  /// @param iWidth Width of bounding box.
  /// @param iAlphaColor Interpolation parameter: alpha of the new
  ///        color model will be added to (1-alpha) of the old model.
  /// @param iSkinThreshold Threshold parameter for finding skin
  ///        pixels using the old skin model. See ComputeSkinMask
  ///        for details.
  void cvp_UpdateSkinColor(cvp_SkinColourModel& iopSkinColor,
          IplImage* ipImage, int iY, int iX, int iHeight,
			   int iWidth, real iAlphaColor, real iSkinThreshold);

} // namespace OpenCvPlus

#endif
