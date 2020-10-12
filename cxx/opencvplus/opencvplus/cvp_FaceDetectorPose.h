/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorPose.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector pose definitions
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORPOSE_H__
#define __CVP_FACEDETECTORPOSE_H__

namespace OpenCvPlus {

/// @brief Represents orientation of detected faces for a given face detector.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

enum cvp_FaceDetectorPose {
    /// Frontal faces
    CVP_FACEDETECTOR_FACE = 1,
    /// Left profile faces
    CVP_FACEDETECTOR_PROFILE_LEFT = 2,
    /// Right profile faces
    CVP_FACEDETECTOR_PROFILE_RIGHT = 3,
    /// Faces of unknown orientation
    CVP_FACEDETECTOR_POSE_UNKNOWN = 10
};

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORPOSE_H__
