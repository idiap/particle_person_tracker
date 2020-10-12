/**
 * @file cxx/bayes_image/bicv_TrainingData.h
 * @date 23 October 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Represents annotated data used for training HOG models
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_TRAININGDATA_H__
#define __BICV_TRAININGDATA_H__

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPose.h>                 // head pose

namespace BICV {

/// @brief Represents the annotation data available for a training exemplar.
///
/// This class contains annotation data that is used when performing training
/// of various head pose models (HOG / skin / ...)
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    23.10.2012

struct bicv_TrainingData {

    /// Head pose of the training exemplar
    OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose m_HeadPose;

    /// Image coordinates of the face for the training exemplar
    CvRect m_FaceRect;

    /// Image coordinates of the left eye for the training exemplar
    CvPoint m_LeftEye;

    /// Image coordinates of the right eye for the training exemplar
    CvPoint m_RightEye;

    /// ID of a person to which this training data relates
    int m_PersonId;

    /// ID of a session for a person to which this training data relates
    int m_SessionId;

}; // bicv_TrainingData

} // namespace BICV

#endif // __BICV_TRAININGDATA_H__
