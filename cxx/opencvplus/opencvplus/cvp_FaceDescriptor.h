/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDescriptor.h
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

#ifndef __CVP_FACEDESCRIPTOR_H__
#define __CVP_FACEDESCRIPTOR_H__

// SYSTEM INCLUDES
#include <cv.h>                                         // for CvRect
#include <boost/date_time/posix_time/posix_time.hpp>    // boost posix time

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetectorPose.h>            // for FD pose
#include <opencvplus/cvp_HeadPose.h>

namespace OpenCvPlus {

/// @brief Represents detected face: its orientation as defined by detector,
/// its location on an image.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

struct cvp_FaceDescriptor {
  /// Face location on an image
  CvRect m_FaceRegion;
  /// Image width
  int m_ImageWidth;
  /// Image height
  int m_ImageHeight;
  /// Detection time
  boost::posix_time::ptime m_DetectionTime;
  /// Pose, as defined by face detector
  cvp_FaceDetectorPose m_Pose;

  /// Head Pose set by the face detector (if applicable, OpenHeadPose)
  cvp_HeadPose<> m_HeadPose;
};

/// Global output streaming operator defined for face descriptors
std::ostream&
operator<<(std::ostream& out_stream, const cvp_FaceDescriptor& face_descriptor);

} // namespace OpenCvPlus

#endif // __CVP_FACEDESCRIPTOR_H__
