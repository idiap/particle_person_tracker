/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 *
 */
#ifndef __CVP_FACEDETECTOROPENPOSE_H__
#define __CVP_FACEDETECTOROPENPOSE_H__

#include <cv.h>
#include "opencvplus/cvp_FaceDetector.h"

#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>
#include <openpose/face/faceDetector.hpp>

namespace OpenCvPlus {

class cvp_FaceDetectorOpenPose: public cvp_FaceDetector
{
public:
  cvp_FaceDetectorOpenPose(const std::string& model_folder,
                           const std::string& pose_model = "COCO",
                           const std::string& net_resolution = "-1x128",
                           int visu = 0,
                           int min_face_height = 15);

  virtual ~cvp_FaceDetectorOpenPose();

protected:

  virtual void prepare_internal(IplImage *image);

  virtual void detect_faces_internal(IplImage * image,
                                     const boost::posix_time::ptime& time,
                                     std::list<cvp_FaceDescriptor>& storage);

protected:
  std::string m_model_name;
  int m_net_resolution;
  int m_visu;
  bool m_is_initialised;
  int m_min_face_height;
  // int m_net_resolution_w;

  // Openpose temporary variable to avoid recreating them all the time
  // in detect_faces_internal. See
  //
  //    openpose/examples/tutorial_pose/1_extract_from_image.cpp from
  //
  // openpose repository on
  //
  //    https://github.com/CMU-Perceptual-Computing-Lab/openpose
  //
  // for a simple example on how to apply simply the detector on an
  // input image.
  op::PoseModel m_pose_model;

  op::CvMatToOpInput        cvMatToOpInput;
  op::CvMatToOpOutput       cvMatToOpOutput;
  op::OpOutputToCvMat       opOutputToCvMat;
  op::ScaleAndSizeExtractor scaleAndSizeExtractor;
  std::shared_ptr<op::PoseExtractorCaffe> poseExtractorCaffe;
  std::shared_ptr<op::PoseGpuRenderer>    poseRenderer;
  // op::FaceDetector          faceDetector; // Not used because too large faces
};

} // namespace OpenCvPlus

#endif
