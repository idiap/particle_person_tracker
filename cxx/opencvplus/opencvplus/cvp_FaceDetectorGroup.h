/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorGroup.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector group to work with several face detectors simultaneously
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORGROUP_H__
#define __CVP_FACEDETECTORGROUP_H__

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetectorFactory.h>          // FD factory
#include <opencvplus/cvp_FaceDetectorPose.h>             // FD pose
#include <opencvplus/cvp_FaceDescriptor.h>               // detection info

namespace OpenCvPlus {

/// @brief Face detector group to define and work with several face detectors
/// simultaneously
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    26.11.2012

class cvp_FaceDetectorGroup
{
public:

  /// Constructor.
  ///@param face_detector_types A list of face detector types that define
  /// face detectors to construct within the group
  ///@param vm Map contatining config variables values
  cvp_FaceDetectorGroup(const std::list<cvp_FaceDetectorType>& face_detector_types,
                        const boost::program_options::variables_map& vm);

  /// Destructor.
  ~cvp_FaceDetectorGroup();

  // OPERATIONS

  /// Method to be called to prepare face detectors within this group
  /// to work with images of a certain type and image fractions.
  /// Calls prepare methods in
  /// individual detectors to perform initialisation and storage allocation.
  /// Images passed to the detect_faces method should have the same type
  /// as the one provided to this method, otherwise a runtime error
  /// could occur.
  /// @param image Example of image this detector would deal with.
  /// @param area_fraction Area fraction to be used for face detection,
  ///         defaults to 1.0
  void prepare(IplImage * image, float area_fraction = 1.0f);

  /// Detect faces on a given image. All detectors are called to detect
  /// faces on a given image, faces are cached and can be further retrieved
  /// by calling cached_faces method.
  /// @param image Image to detect faces on
  /// @param time Timestamp of the image, to be assigned to face detections
  /// @return A list of face descriptors.
  const std::list<cvp_FaceDescriptor>& detect_faces(IplImage * image,
                                                    const boost::posix_time::ptime& time);

  /// Detect faces on a given image using only one detector defined by an
  /// iterator (current detector index). The faces are cached and can be
  /// further retrieved by calling cached_faces method.
  /// @param image Image to detect faces on
  /// @param time Timestamp of the image, to be assigned to face detections
  /// @return A list of face descriptors.
  const std::list<cvp_FaceDescriptor>&
  detect_faces_next(IplImage * image, const boost::posix_time::ptime& time);

  /// Return cached faces stored from the last detection.
  /// @return A list of cached face descriptors.
  const std::list<cvp_FaceDescriptor>& cached_faces() const {
    return m_Faces;
  }

  /// Return cached bodies
  /// @return A list of cached body descriptors.
  const std::list<cvp_BodyDescriptor>& cached_bodies() const {
    return m_Bodies;
  }

  /// Clears cached faces.
  void invalidate();

private:
  // collection of face detectors
  std::vector<cvp_FaceDetector*> m_FaceDetectors;
  // cache - list of previously detected faces
  std::list<cvp_FaceDescriptor> m_Faces;
  // cache - list of previously detected bodies
  std::list<cvp_BodyDescriptor> m_Bodies;
  // current face detector index used in detect_faces_next method
  unsigned m_CurrentFaceDetectorIndex;

  bool m_IsPrepared;

};

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORGROUP_H__
