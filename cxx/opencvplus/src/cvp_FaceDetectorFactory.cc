// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvp_FaceDetectorFactory - factory to create face detectors
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include "opencvplus/cvp_FaceDetectorFactory.h"   // declaration of this
#include "opencvplus/cvp_FaceDetectorCV.h"        // OpenCV face detector
#include "opencvplus/cvp_Exceptions.h"            // CVP exceptions definitions

#ifdef __OPENPOSE_FOUND__
#include "opencvplus/cvp_FaceDetectorOpenPose.h"
#endif

#ifdef __OPENHEADPOSE_FOUND__
#include "opencvplus/cvp_FaceDetectorOpenHeadPose.h"
#endif

using namespace std;

namespace OpenCvPlus {

/////////////////////////////// PUBLIC ///////////////////////////////////////

const string cvp_FaceDetectorFactory::OPT_FD_OPENCV_FACE_CASCADE_STR =
    "fd_opencv_face_cascade";
const string cvp_FaceDetectorFactory::OPT_FD_OPENCV_PROFILE_LEFT_CASCADE_STR =
    "fd_opencv_profile_left_cascade";
const string cvp_FaceDetectorFactory::OPT_FD_OPENCV_PROFILE_RIGHT_CASCADE_STR =
    "fd_opencv_profile_right_cascade";

// OpenPose
const string cvp_FaceDetectorFactory::OPT_FD_OP_POSE_STR =
  "fd_op_pose";

const string cvp_FaceDetectorFactory::OPT_FD_OP_MODEL_STR =
  "fd_op_model";

const string cvp_FaceDetectorFactory::OPT_FD_OP_NET_RESOLUTION_STR =
  "fd_op_net_resolution";

const string cvp_FaceDetectorFactory::OPT_FD_OP_VISU_STR =
  "fd_op_visu";

// OpenHeadPose
const string cvp_FaceDetectorFactory::OPT_FD_OHP_MODEL_STR =
  "fd_ohp_model";

const string cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_STR =
   "fd_min_face_height";

const string cvp_FaceDetectorFactory::OPT_FD_MIN_FACE_HEIGHT_RATIO_STR =
   "fd_min_face_height_ratio";

#ifdef __RSBINTEGRATION_FOUND__
bool registerConverter = true;
#endif

/* static */
cvp_FaceDetector * cvp_FaceDetectorFactory::create_face_detector(
        cvp_FaceDetectorType face_detector_type,
        const boost::program_options::variables_map& vm){

    cvp_FaceDetector * face_detector = 0;

    switch(face_detector_type) {

    case CVP_FACEDETECTOR_OPENCV_FACE:
        face_detector = new cvp_FaceDetectorCV(
                vm[OPT_FD_OPENCV_FACE_CASCADE_STR].as<string>(),
                CVP_FACEDETECTOR_FACE);
        break;

    case CVP_FACEDETECTOR_OPENCV_PROFILE_LEFT:
        face_detector = new cvp_FaceDetectorCV(
                vm[OPT_FD_OPENCV_PROFILE_LEFT_CASCADE_STR].as<string>(),
                CVP_FACEDETECTOR_PROFILE_LEFT);
        break;

    case CVP_FACEDETECTOR_OPENCV_PROFILE_RIGHT:
        face_detector = new cvp_FaceDetectorCV(
                vm[OPT_FD_OPENCV_PROFILE_RIGHT_CASCADE_STR].as<string>(),
                CVP_FACEDETECTOR_PROFILE_RIGHT);
        break;

    case CVP_FACEDETECTOR_OPENPOSE:
#ifdef __OPENPOSE_FOUND__
      face_detector =
        new cvp_FaceDetectorOpenPose(vm[OPT_FD_OP_MODEL_STR].as<string>(),
                                     vm[OPT_FD_OP_POSE_STR].as<string>(),
                                     vm[OPT_FD_OP_NET_RESOLUTION_STR].as<string>(),
                                     vm[OPT_FD_OP_VISU_STR].as<int>(),
                                     vm[OPT_FD_MIN_FACE_HEIGHT_STR].as<int>());
      break;
#else
        throw OpenCvPlus::cvp_Exception("OpenPose face detector "
                                        "was not compiled, "
                                        "change build options!");
#endif

    case CVP_FACEDETECTOR_OPENHEADPOSE:
#ifdef __OPENHEADPOSE_FOUND__
      face_detector =
        new cvp_FaceDetectorOpenHeadPose(vm[OPT_FD_OP_MODEL_STR].as<string>(),
                                         vm[OPT_FD_OHP_MODEL_STR].as<string>(),
                                         vm[OPT_FD_OP_POSE_STR].as<string>(),
                                         vm[OPT_FD_OP_NET_RESOLUTION_STR].as<string>(),
                                         vm[OPT_FD_OP_VISU_STR].as<int>(),
                                         vm[OPT_FD_MIN_FACE_HEIGHT_STR].as<int>());
      break;
#else
        throw OpenCvPlus::cvp_Exception("OpenHeadPose face detector "
                                        "was not compiled, "
                                        "change build options!");
#endif

    default:
        throw OpenCvPlus::cvp_Exception("Unknown face detector type requested!");
    }

    return face_detector;
}

} // namespace OpenCvPlus
