/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef _CVP_FACEDETECTOROPENHEADPOSE_H_
#define _CVP_FACEDETECTOROPENHEADPOSE_H_

#include <cv.h>
#include "opencvplus/cvp_FaceDetector.h"

#include "openheadpose/Config.h"
#include "openheadpose/PoseEstimator.h"

namespace OpenCvPlus {

class cvp_FaceDetectorOpenHeadPose: public cvp_FaceDetector
{
public:
  cvp_FaceDetectorOpenHeadPose(const std::string& openpose_model,
                               const std::string& openheadpose_model,
                               const std::string& pose_model = "COCO",
                               const std::string& net_resolution = "-1x128",
                               int visu = 0,
                               int min_face_height = 15);

  virtual ~cvp_FaceDetectorOpenHeadPose();

protected:

  virtual void prepare_internal(IplImage * image);

  virtual void detect_faces_internal(IplImage * image,
                                     const boost::posix_time::ptime& time,
                                     std::list<cvp_FaceDescriptor>& storage);

protected:
  std::string m_openpose_model_folder;
  std::string m_openheadpose_model_folder;
  std::string m_pose_model;
  std::string m_net_resolution;
  int m_visu;
  int m_min_face_height;
  ohp::PoseEstimator *m_pose_estimator;

  // bool m_is_initialised;
  // std::string m_model_name;
  // int m_net_resolution_h;
  // int m_net_resolution_w;
};

} // namespace OpenCvPlus



#endif /* _CVP_FACEDETECTOROPENHEADPOSE_H_ */
