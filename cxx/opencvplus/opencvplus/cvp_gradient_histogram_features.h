/**
 * Functions for computing and comparing histogram of gradient
 * features. A histogram of gradients represents the frequency with
 * which each image gradient angle appears within a block of pixels.
 *
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_HISTOGRAM_FEATURES_H__
#define __CVP_HISTOGRAM_FEATURES_H__

// LOCAL INCLUDES
#include "cvplus.h"
#include "cvp_HistogramTemplate.h"
#include "cvp_HistogramTemplateSimple.h"

namespace OpenCvPlus {

  // Type and constant definitions

  /// Constants for choosing the histogram distance metric, used when
  /// comparing two histogram templates.
  ///   CVP_HIST_DISTANCE_CORREL The correlation method from OpenCV.
  ///   CVP_HIST_DISTANCE_CHISQR The chi-square method from OpenCV.
  ///   CVP_HIST_DISTANCE_INTERSECT The intersection method from OpenCV *inverted*.
  ///   CVP_HIST_DISTANCE_BHATACHARYYA The Bhatacharyya distance (not from OpenCV).
  ///   CVP_HIST_DISTANCE_HAUSSDORF The Haussdorf distance.
  ///   CVP_HIST_DISTANCE_FGBG Unknown. Not implemented.
  typedef enum cvp_HistogramDistance {
    CVP_HIST_DISTANCE_CORREL = CV_COMP_CORREL,
    CVP_HIST_DISTANCE_CHISQR = CV_COMP_CHISQR,
    CVP_HIST_DISTANCE_INTERSECT = CV_COMP_INTERSECT,
    CVP_HIST_DISTANCE_BHATACHARYYA,
    CVP_HIST_DISTANCE_HAUSSDORF,
    CVP_HIST_DISTANCE_FGBG
  } cvp_HistogramDistance;

  /// Constants for choosing the histogram distance metric, used when
  /// comparing two simple histogram templates.
  ///   CVP_HIST_DISTANCE_EUCLIDEAN
  ///     \sum_{h=1}^{histograms} \sqrt{ \sum_{b=1}^{bins} (this[h][b] - other[h][b])^2 }
  ///   CVP_HIST_DISTANCE_MAHALANOBIS Not implemented.
  ///   CVP_HIST_DISTANCE_L1 This is *not* the L1 distance.
  ///     See the bug comment in cvp_HistogramTemplateSimple.cc
  typedef enum cvp_HistogramDistanceSimple {
    CVP_HIST_DISTANCE_EUCLIDEAN = 1,
    CVP_HIST_DISTANCE_MAHALANOBIS = 2,
    CVP_HIST_DISTANCE_L1 = 3
  } cvp_HistogramDistanceSimple;


  // Prototypes to be defined later

  class cvp_HistogramTemplate;
  class cvp_HistogramTemplateSimple;


  // Function definitions

  /// Compute the distance between two histogram templates.
  /// @param iH1 Pointer to the first histogram template.
  /// @param iH2 Pointer to the second histogram template.
  /// @param iMethod The distance metric. See the cvp_HistogramDistance enum
  ///        for details. Default: CVP_HIST_DISTANCE_BHATACHARYYA.
  real cvp_CompareHistogramTemplates(const cvp_HistogramTemplate* ipH1,
				      const cvp_HistogramTemplate* ipH2,
				      cvp_HistogramDistance iMethod = CVP_HIST_DISTANCE_BHATACHARYYA);

  /// Compute the distance between two simple histogram templates.
  /// @param iH1 Pointer to the first simple histogram template.
  /// @param iH2 Pointer to the second simple histogram template.
  /// @param iMethod The distance metric. See the cvp_HistogramDistanceSimple
  ///        enum for details. Default: CVP_HIST_DISTANCE_EUCLIDEAN.
  /// @todo Add const qualifiers to iH1 and iH2 arguments.
  real cvp_CompareSimpleHistogramTemplates(const cvp_HistogramTemplateSimple* ipH1,
					    const cvp_HistogramTemplateSimple* ipH2,
					    cvp_HistogramDistanceSimple iMethod = CVP_HIST_DISTANCE_EUCLIDEAN);

  /// Compute integral image histograms of the gradient angle at each
  /// location in the input image.
  /// @param ipGradientMagnitude Floating point image of the gradient
  ///        magnitude.
  /// @param ipGradientAngle Floating point image of the gradient direction.
  /// @param opIntegralGradientHistogram Output: array of integral images,
  ///        where each image in the array represents on histogram bin, and is
  ///        a floating point integral image of the weighted number of times
  ///        that the angles in that bin were observed in the input image.
  ///        The gradient magnitude, and interpolation values in the histogram
  ///        bin look-up table are used as weights.
  /// @param iNumberOfBins The number of bins in the histogram.
  void cvp_ComputeIntegralGradientHistogram(IplImage const* ipGradientMagnitude,
					    IplImage const* ipGradientAngle,
					    CvMat** opIntegralGradientHistogram,
					    int iNumberOfBins);

  /// Compute gradient x-component, y-component, magnitude and orientation of
  /// an image.
  /// @param ipFloatImage Floating point input image.
  /// @param opGradientX Output float image: gradient x-component.
  /// @param opGradientYOutput float image: gradient y-component.
  /// @param opGradientMagnitude Output float image: gradient magnitude.
  /// @param opGradientAngle Output float image: gradient angle.
  /// @param iMagnitudeCap Gradient magnitude is capped at this value.
  void cvp_ComputeGradients(const IplImage* ipFloatImage, IplImage* opGradientX,
			    IplImage* opGradientY, IplImage* opGradientMagnitude,
			    IplImage* opGradientAngle, real iMagnitudeCap);

  /// Use an integral image histogram to sum the histograms within an image
  /// block, defined by the edges of a bounding box.
  /// @param iTopY Top edge of the bounding box; non-negative.
  /// @param iLeftX Left edge of the bounding box; non-negative.
  /// @param iBottomY Bottom edge of the bounding box; non-negative.
  /// @param iRightX Right edge of the bounding box; non-negative.
  /// @param ipIntegralGradientHistogram Integral image histogram.
  /// @param oHistogram Storage for the summed histogram counts.
  /// @param iNumberOfBins The number of bins in the histograms.
  void cvp_ComputeSumOfGradientHistograms(int iTopY, int iLeftX, int iBottomY, int iRightX,
					  CvMat const* const* ipIntegralGradientHistogram,
					  real* opHistogram, int iNumberOfBins);

  /// Use an integral image histogram to sum the histograms within all
  /// blocks in an image.
  /// @param iImageHeight The number of rows in the input image
  /// @param iImageWidth The number of column in the input image
  /// @param opHistogram Output storage of length iNumberOfBins * numberOfBlocks
  ///        where numberOfBlocks = iNumberOfColumnBlocks * iNumberOfRowBlocks * iColumnsPerBlock * iRowsPerBlock.
  /// @param opTempHistogram Temporary storage of length iNumberOfBins. Will be
  ///        allocated by the function if NULL.
  /// @param iNumberOfBins The number of bins in the input and output histograms.
  /// @param ipIntegralGradientHistogram Integral image histogram.
  /// @param iNumberOfColumnBlocks Number of large blocks in the image, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in the image, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
  void cvp_ComputeBlockGradientHistogram(
    int iImageHeight, int iImageWidth,
    OpenCvPlus::real* opHistogram, OpenCvPlus::real* opTempHistogram,
    int iNumberOfBins, CvMat const* const* ipIntegralGradientHistogram,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock);

  /// Compute the integral image of a singe channel 8-bit image.
  /// @param ipSource The input image.
  /// @param opIntegralImage The destination integral image.
  /// @todo Replace this function with a call to the equivalent (?)
  ///       OpenCV function.
  void cvp_ComputeIntegralImage(IplImage const* ipSource, CvMat* opIntegralImage);

} // namespace OpenCvPlus

#endif
