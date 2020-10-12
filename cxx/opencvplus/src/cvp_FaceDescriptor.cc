/**
 * @file cxx/opencvplus/src/cvp_FaceDescriptor.cc
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

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDescriptor.h>              // declaration of this

namespace OpenCvPlus {

std::ostream&
operator<<(std::ostream& out_stream,
        const cvp_FaceDescriptor& face_descriptor) {
    out_stream << "Face: ";
    switch (face_descriptor.m_Pose) {
    case CVP_FACEDETECTOR_FACE: out_stream << "Frontal, "; break;
    case CVP_FACEDETECTOR_PROFILE_LEFT: out_stream << "Left, "; break;
    case CVP_FACEDETECTOR_PROFILE_RIGHT: out_stream << "Right, "; break;
    case CVP_FACEDETECTOR_POSE_UNKNOWN: out_stream << "Unknown, "; break;
    }
    out_stream << "BBox(" <<
            face_descriptor.m_FaceRegion.x << ", " <<
            face_descriptor.m_FaceRegion.y << ", " <<
            face_descriptor.m_FaceRegion.width << ", " <<
            face_descriptor.m_FaceRegion.height << "), " <<
            " image size (" << face_descriptor.m_ImageWidth << ", " <<
            face_descriptor.m_ImageHeight << "), " <<
            " time (" << face_descriptor.m_DetectionTime << ")";
    return out_stream;
} // operator<<

} // namespace OpenCvPlus
