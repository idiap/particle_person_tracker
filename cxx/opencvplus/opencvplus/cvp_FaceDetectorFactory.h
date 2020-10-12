// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvp_FaceDetectorFactory - factory to create face detectors
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __CVP_FACEDETECTORFACTORY_HPP__
#define __CVP_FACEDETECTORFACTORY_HPP__

// SYSTEM INCLUDES
#include <boost/program_options.hpp>                // boost program options

// LOCAL INCLUDES
#include "cvp_FaceDetector.h"                       // general face detector

namespace OpenCvPlus {

enum cvp_FaceDetectorType {

    CVP_FACEDETECTOR_OPENCV_FACE = 1,           // OpenCV face detector
    CVP_FACEDETECTOR_OPENCV_PROFILE_LEFT = 2,   // OpenCV face detector
    CVP_FACEDETECTOR_OPENCV_PROFILE_RIGHT = 3,  // OpenCV face detector

    CVP_FACEDETECTOR_CTU = 4,                   // CTU face detector
    CVP_FACEDETECTOR_TORCH = 5,                 // Torch3 face detector
    CVP_FACEDETECTOR_RSB_FACE = 6,              // RSB face detector
    CVP_FACEDETECTOR_RSB_PROFILE_LEFT = 7,      // RSB face detector
    CVP_FACEDETECTOR_RSB_PROFILE_RIGHT = 8,     // RSB face detector

    CVP_FACEDETECTOR_UNKNOWN = 10,              // reserved for int -> enum casts

    CVP_FACEDETECTOR_OPENPOSE = 11,             // OpenPose face detector
    CVP_FACEDETECTOR_OPENHEADPOSE = 12,         // OpenPose face detector
};

inline cvp_FaceDetectorType str2faceDetectorType(const std::string& name)
{
  std::cout << "name " << name << std::endl;
  if     (name=="cv")           return CVP_FACEDETECTOR_OPENCV_FACE;
  else if(name=="cv-l")         return CVP_FACEDETECTOR_OPENCV_PROFILE_LEFT;
  else if(name=="cv-r")         return CVP_FACEDETECTOR_OPENCV_PROFILE_RIGHT;
  else if(name=="ctu")          return CVP_FACEDETECTOR_CTU;
  else if(name=="torch")        return CVP_FACEDETECTOR_TORCH;
  else if(name=="rsb")          return CVP_FACEDETECTOR_RSB_FACE;
  else if(name=="rsb-l")        return CVP_FACEDETECTOR_RSB_PROFILE_LEFT;
  else if(name=="rsb-r")        return CVP_FACEDETECTOR_RSB_PROFILE_RIGHT;
  else if(name=="openpose")     return CVP_FACEDETECTOR_OPENPOSE;
  else if(name=="openheadpose") return CVP_FACEDETECTOR_OPENHEADPOSE;
  else                          return CVP_FACEDETECTOR_UNKNOWN;
}


/// @brief Factory to produce face detectors
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class cvp_FaceDetectorFactory {

public:

  // OpenCV face detector cascade options
  static const std::string OPT_FD_OPENCV_FACE_CASCADE_STR;
  static const std::string OPT_FD_OPENCV_PROFILE_LEFT_CASCADE_STR;
  static const std::string OPT_FD_OPENCV_PROFILE_RIGHT_CASCADE_STR;

  // OpenPose
  static const std::string OPT_FD_OP_POSE_STR; // COCO or BODY_25
  static const std::string OPT_FD_OP_MODEL_STR; // Should be "/path/to/openpose/models/"
  static const std::string OPT_FD_OP_NET_RESOLUTION_STR;
  static const std::string OPT_FD_OP_VISU_STR;

  // OpenHeadPose
  static const std::string OPT_FD_OHP_MODEL_STR; // Should be "/path/to/openheadpose/models/"

  static const std::string OPT_FD_MIN_FACE_HEIGHT_STR;
  static const std::string OPT_FD_MIN_FACE_HEIGHT_RATIO_STR;

  static cvp_FaceDetector*
  create_face_detector(cvp_FaceDetectorType face_detector_type,
                       const boost::program_options::variables_map& vm);

private:

  // LIFECYCLE

  /// Constructor
  cvp_FaceDetectorFactory();

};

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORFACTORY_HPP__
