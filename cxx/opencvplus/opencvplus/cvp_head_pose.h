/**
 * Functions for training, loading and saving a discrete head pose
 * model based on histogram-of-gradient features and a skin color
 * mask.
 *
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * See COPYING file for the complete license text.
 */

#ifndef __CVP_HEAD_POSE_H__
#define __CVP_HEAD_POSE_H__

#include <string>

#include "cvplus.h"
#include "cvp_HistogramTemplateSimple.h"
#include "cvp_HeadPoseDiscreteDomain.h"
#include "cvp_SkinColourModel.h"

namespace OpenCvPlus {

  /// Compute the mean histogram template from training images of faces.
  /// @param iPath The path in which the training index file (index.txt) lives.
  /// @param iopTrainedMeanHistograms Output storage.
  /// @param ipPoseActive Boolean map from head pose indices, indicating which
  ///        poses should be modelled.
  /// @param iMaxNumberOfPoses The length of the ipPoseActive array.
  /// @param iMagnitudeCap The gradient magnitude is capped at this value.
  /// @param iNumberOfColumnBlocks Number of large blocks in images, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in images, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
void cvp_trainMultiScaleGradientHistograms(
  std::string iPath, cvp_HistogramTemplateSimple* const* iopTrainedMeanHistograms,
  int const* ipPoseActive, int iMaxNumberOfPoses, real iMagnitudeCap,
  int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock);

  /// Compute the reference skin color mask from training images of faces.
  /// @param iPath The path in which the training index file (index.txt) lives.
  /// @param ipPoseActive Boolean map from head pose indices, indicating which
  ///        poses should be modelled.
  /// @param iMaxNumberOfPoses The length of the ipPoseActive array.
  /// @param ipSkinColor A length-5 array with [mean green, mean red, variance
  ///                    green, variance red, covariance green-red] statistics.
  /// @param iSkinThreshold Threshold for deciding whether a pixel is skin-
  ///                       colored or not. See iDetectionThreshold in
  ///                       cvp_ComputeSkinMask() for details.
  /// @param iNumberOfColumnBlocks Number of large blocks in images, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in images, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
  /// @param opTrainedSkinMasks Output storage.
  void cvp_trainSkinMask(
    std::string iPath, const cvp_HeadPoseDiscreteDomain& head_pose_domain,
    const cvp_SkinColourModel&  ipSkinColor, float iSkinThreshold,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock,
    cvp_HistogramTemplateSimple* const* opTrainedSkinMasks);

  /// Load a save head pose gradient histogram and skin mask model
  /// from file.
  /// @param opGradientHistogramTemplate Storage for the gradient histograms.
  /// @param opSkinMaskTemplate Storage for the skin mask.
  /// @param ipPoseActive Boolean map from head pose indices, indicating which
  ///        poses should be modelled.
  /// @param iMaxNumberOfPoses The length of the ipPoseActive array.
  /// @param iNumberOfGradientBins Number of bins in each gradient histogram.
  /// @param iNumberOfColumnBlocks Number of large blocks in images, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in images, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
  void cvp_LoadHeadPoseModel(
    cvp_HistogramTemplateSimple* const* opGradientHistogramTemplate,
    cvp_HistogramTemplateSimple* const* opSkinMaskTemplate,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain,
    int iNumberOfGradientBins, int iNumberSplitBlockColumns,
    int iNumberSplitBlockRows, int iNumberSplitColumns, int iNumberSplitRows);

  /// Save a head pose gradient histogram and skin mask model to file.
  /// @param ipGradientHistogramTemplate Gradient histograms to save.
  /// @param ipSkinMaskTemplate Skin mask to save.
  /// @param ipPoseActive Boolean map from head pose indices, indicating which
  ///        poses should be modelled.
  /// @param iMaxNumberOfPoses The length of the ipPoseActive array.
  /// @param iNumberOfGradientBins Number of bins in each gradient histogram.
  /// @param iNumberOfColumnBlocks Number of large blocks in images, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in images, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
  void cvp_SaveHeadPoseModel(
    cvp_HistogramTemplateSimple const* const* ipGradientHistogramTemplate,
    cvp_HistogramTemplateSimple const* const* ipSkinMaskTemplate,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain, int iNumberOfGradientBins,
    int iNumberSplitBlockColumns, int iNumberSplitBlockRows, int iNumberSplitColumns, int iNumberSplitRows);

} // namespace OpenCvPlus

#endif
