/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_FaceDetectorOpenHeadPose.h"
#include "opencvplus/cvp_OpenPoseUtils.h"
#include "openheadpose/Utils.h"

namespace OpenCvPlus {

const std::string OPENHEADPOSE_WINDOW = "Idiap OpenHeadPose";

cvp_FaceDetectorOpenHeadPose::
cvp_FaceDetectorOpenHeadPose(const std::string& openpose_model,
                             const std::string& openheadpose_model,
                             const std::string& pose_model,
                             const std::string& net_resolution,
                             int visu, int min_face_height):
  cvp_FaceDetector(CVP_FACEDETECTOR_FACE),
  m_openpose_model_folder(openpose_model),
  m_openheadpose_model_folder(openheadpose_model),
  m_pose_model(pose_model),
  m_net_resolution(net_resolution),
  m_visu(visu),
  m_min_face_height(min_face_height),
  m_pose_estimator(NULL)
{
  std::cout << "[FaceDetectorOpenHeadPose]"
            << " model " << m_pose_model
            << " resolution " << net_resolution
            << std::endl;
  std::cout << "[FaceDetectorOpenHeadPose]"
            << " OpenPose folder " << openpose_model
            << std::endl;
  std::cout << "[FaceDetectorOpenHeadPose]"
            << " OpenHeadPose folder " << openheadpose_model
            << std::endl;

  if(m_visu>0)
    {
      cv::namedWindow(OPENHEADPOSE_WINDOW, cv::WINDOW_NORMAL);
      // const int kw = 480;
      // const int kh = 260;
      // cv::resizeWindow(OPENHEADPOSE_WINDOW, 4*kw, 4*kh);
      // cv::moveWindow(OPENHEADPOSE_WINDOW, 0, 0);
    }

}


cvp_FaceDetectorOpenHeadPose::
~cvp_FaceDetectorOpenHeadPose()
{
  if(m_pose_estimator)
    {
      delete m_pose_estimator;
      m_pose_estimator = 0;
    }
}


void
cvp_FaceDetectorOpenHeadPose::
prepare_internal(IplImage *image)
{
}


void
cvp_FaceDetectorOpenHeadPose::
detect_faces_internal(IplImage * image,
                      const boost::posix_time::ptime& time,
                      std::list<cvp_FaceDescriptor>& storage)
{
  if(!m_pose_estimator)
    {
      ohp::Config config;
      config.openposeModelFolder = m_openpose_model_folder;
      config.openheadposeModelFolder = m_openheadpose_model_folder;
      config.poseModel = m_pose_model;
      config.netInputSize = m_net_resolution;
      if(m_visu>0)
        {
          config.drawOpenPose = true;
          config.drawHeadPose = true;
        }
      m_pose_estimator = new ohp::PoseEstimator(config);
    }

  std::list<cvp_FaceDescriptor> raw_faces;
  std::list<cvp_BodyDescriptor> raw_bodies;
  cv::Mat inputImage = cv::cvarrToMat(image);

  const op::PoseModel poseModel = op::flagsToPoseModel(op::String(m_pose_model));

  m_pose_estimator->estimate(inputImage);

  std::vector<ohp::Pose> poses = m_pose_estimator->getPoses();

  if(m_visu>0)
    {
      cv::Mat outputImage = m_pose_estimator->getRenderOutput();
      cv::imshow(OPENHEADPOSE_WINDOW, outputImage);
    }

  for(size_t i=0 ; i<poses.size() ; ++i)
    {
      const std::vector<float>& keypoints = poses[i].keypoints();
      std::vector<double> bb =
        keypoints_to_bounding_box(keypoints,
                                  poseModel,
                                  0); // keypoints contains 1 person only

      cvp_FaceDescriptor temp_fd;
      temp_fd.m_ImageWidth = image->width;
      temp_fd.m_ImageHeight = image->height;

      temp_fd.m_FaceRegion.x = bb[0];
      temp_fd.m_FaceRegion.y = bb[1];
      temp_fd.m_FaceRegion.width  = bb[2];
      temp_fd.m_FaceRegion.height = bb[3];

      ////////////////////////////////////////////////////////////
      // Output of bounding box regressor
      ////////////////////////////////////////////////////////////
      // temp_fd.m_FaceRegion.x = poses[i].headX();
      // temp_fd.m_FaceRegion.y = poses[i].headY();
      // temp_fd.m_FaceRegion.width  = poses[i].headWidth();
      // temp_fd.m_FaceRegion.height = poses[i].headHeight();
      ////////////////////////////////////////////////////////////

      temp_fd.m_DetectionTime = time;
      temp_fd.m_Pose = pose();

      // pan = yaw, tilt = pitch
      temp_fd.m_HeadPose.roll(poses[i].roll());
      temp_fd.m_HeadPose.pan(poses[i].yaw());
      temp_fd.m_HeadPose.tilt(poses[i].pitch());
      // std::cout << "YAW " << poses[i].yaw()/M_PI*180.0 << std::endl;
      // std::cout << i << " pose " << temp_fd.m_HeadPose << std::endl;

      // Skip small faces
      if(bb[2]>=m_min_face_height && bb[3]>=m_min_face_height)
      {
        raw_faces.push_back(temp_fd);
        cvp_BodyDescriptor bd = keypoints_to_body(keypoints, poseModel, 0);
        raw_bodies.push_back(bd);
      }
    }

  correct_negative_coordinates(raw_faces, image->width, image->height);

  std::copy(raw_faces.begin(), raw_faces.end(), back_inserter(storage));
  std::copy(raw_bodies.begin(), raw_bodies.end(), back_inserter(mBodies));

}


} // namespace OpenCvPlus
