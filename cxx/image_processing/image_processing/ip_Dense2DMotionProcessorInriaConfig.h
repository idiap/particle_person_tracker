/**
 * @file cxx/image_processing/image_processing/ip_Dense2DMotionProcessorInriaConfig.h
 * @date 05 December 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Configuration options for Inria dense motion processor
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_DENSE2DMOTIONPROCESSORINRIACONFIG_H__
#define __IP_DENSE2DMOTIONPROCESSORINRIACONFIG_H__

// SYSTEM INCLUDES

#include <string>

// PROJECT INCLUDES
#include <cmotion2d/CMotion2DModel.h>             // 2D motion model
#include <cmotion2d/CMotion2DEstimator.h>         // 2D motion estimator
#include <cmotion2d/CMotion2DWarping.h>           // 2D motion warping
#include <cmotion2d/CMotion2DImage.h>             // 2D motion image
#include <image_processing/ip_Dense2DMotionProcessor.h>  // base class

namespace ImageProcessing {

/// @brief Configuration options for CMotion2D dense motion estimation code
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    05.12.2012

struct ip_Dense2DMotionProcessorInriaConfig {

    /// Model ID to use for motion estimation
    std::string m_ModelId;
    /// Flag indicating whether illumination variation should be estimated
    bool m_EstimateIlluminationVariationFlag;
    /// Number of levels in pyramides
    int m_PyramidLevelsNum;
    /// Flag indicating whether robust estimation should be used
    bool m_RobustEstimationFlag;
    /// Robust function type (Tukey / Cauchy / Talmar / Welch)
    CMotion2DEstimator::ERobustFunction m_RobustFunctionType;
    /// Scale parameter behaviour (Fixed / Classic / Robust)
    CMotion2DEstimator::ECConstType m_CConstantType;
    /// Scale parameter
    float m_CConstant;
    /// Support percentage for which the estimation is considered as reliable
    float m_ReliableSupportRate;
    /// Number of iterations for reweighted least squares
    int m_NumIterIRLS;
    /// Flag indicating whether support size would should be computed
    bool m_ComputeSupportSizeFlag;
    /// Flag indicating whether the covariance matrix of model parameters
    /// should be computed
    bool m_ComputeCovarianceMatrixFlag;

    static ip_Dense2DMotionProcessorInriaConfig getDefault();

}; // struct ip_Dense2DMotionProcessorInriaConfig

} // namespace ImageProcessing

#endif // __IP_DENSE2DMOTIONPROCESSORINRIACONFIG_H__
