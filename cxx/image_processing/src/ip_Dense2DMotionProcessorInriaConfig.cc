/**
 * @file cxx/image_processing/src/ip_Dense2DMotionProcessorInriaConfig.cc
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

// PROJECT INCLUDES
#include <image_processing/ip_Dense2DMotionProcessorInriaConfig.h>

namespace ImageProcessing {

/* static */ ip_Dense2DMotionProcessorInriaConfig
ip_Dense2DMotionProcessorInriaConfig::getDefault() {

    ip_Dense2DMotionProcessorInriaConfig result;

    result.m_ModelId = "MDL_AFF_TR_ROT_DIV";
    result.m_EstimateIlluminationVariationFlag = false;
    result.m_PyramidLevelsNum = 4;
    result.m_RobustEstimationFlag = true;
    result.m_RobustFunctionType = CMotion2DEstimator::Cauchy;
    result.m_CConstantType = CMotion2DEstimator::CConst_Fixed;
    result.m_CConstant = 5.0f;
    result.m_ReliableSupportRate = 0.2f;
    result.m_NumIterIRLS = 6;
    result.m_ComputeSupportSizeFlag = true;
    result.m_ComputeCovarianceMatrixFlag = true;

    return result;

} // getDefault

} // namespace ImageProcessing
