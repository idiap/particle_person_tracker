/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_OpenPoseUtils.h"
#include <openpose/pose/headers.hpp>

namespace OpenCvPlus {

std::vector<double>
keypoints_to_bounding_box(const std::vector<float>& keypoints,
                          op::PoseModel pose_model,
                          int id)
{
  std::vector<double> bb(4, 0); // x,y,w,h

  const double s_confidence = 0.1;

  const int n_parts = static_cast<int>(op::getPoseNumberBodyParts(pose_model));
  const int n_persons = static_cast<int>(keypoints.size())/n_parts/3;

  if(id >= n_persons) return bb;

  std::vector<std::string> partNames;
  partNames.push_back("Nose");
  partNames.push_back("REye");
  partNames.push_back("LEye");
  partNames.push_back("REar");
  partNames.push_back("LEar");

  std::vector<int> indices;
  for(size_t i=0 ; i<partNames.size() ; ++i)
    indices.push_back(poseBodyPartMapStringToKey(pose_model, partNames[i]));

  const int offset = 3*id*n_parts;
  double xmax = 0.0;
  double xmin = 0.0;
  double ymax = 0.0;
  double ymin = 0.0;

  for(size_t i=0 ; i<indices.size() ; ++i)
    {
      int idx = indices[i];
      double x = keypoints[offset + 3*idx];
      double y = keypoints[offset + 3*idx + 1];;
      double c = keypoints[offset + 3*idx + 2];;

      if(i==0)
        {
          xmin = x;
          xmax = x;
          ymin = y;
          ymax = y;
        }

      if(c>s_confidence)
        {
          if(xmax<x) xmax = x;
          if(xmin>x) xmin = x;
          if(ymax<y) ymax = y;
          if(ymin>y) ymin = y;
        }
    }

  // Compute center as middle of extreme values
  double x = (xmax + xmin)/2.0;
  double y = (ymax + ymin)/2.0;

  double sz = xmax - xmin;
  sz *= 1.2; // + 20%
  bb[0] = x - sz/2.0;
  bb[1] = y - sz/2.0;
  bb[2] = sz;
  bb[3] = sz;

  return bb;
}


// Return cvp_BodyDescriptor for person number "id"
cvp_BodyDescriptor
keypoints_to_body(const std::vector<float>& keypoints,
                  op::PoseModel pose_model,
                  int id)
{
  cvp_BodyDescriptor bd;

  int reye_id = poseBodyPartMapStringToKey(pose_model, "REye");
  int leye_id = poseBodyPartMapStringToKey(pose_model, "LEye");
  int rear_id = poseBodyPartMapStringToKey(pose_model, "REar");
  int lear_id = poseBodyPartMapStringToKey(pose_model, "LEar");
  int nose_id = poseBodyPartMapStringToKey(pose_model, "Nose");
  int neck_id = poseBodyPartMapStringToKey(pose_model, "Neck");
  int lshoulder_id = poseBodyPartMapStringToKey(pose_model, "LShoulder");
  int rshoulder_id = poseBodyPartMapStringToKey(pose_model, "RShoulder");
  int relbow_id = poseBodyPartMapStringToKey(pose_model, "RElbow");
  int lelbow_id = poseBodyPartMapStringToKey(pose_model, "LElbow");
  int rwrist_id = poseBodyPartMapStringToKey(pose_model, "RWrist");
  int lwrist_id = poseBodyPartMapStringToKey(pose_model, "LWrist");
  int rhip_id = poseBodyPartMapStringToKey(pose_model, "RHip");
  int lhip_id = poseBodyPartMapStringToKey(pose_model, "LHip");
  int rknee_id = poseBodyPartMapStringToKey(pose_model, "RKnee");
  int lknee_id = poseBodyPartMapStringToKey(pose_model, "LKnee");
  int rankle_id = poseBodyPartMapStringToKey(pose_model, "RAnkle");
  int lankle_id = poseBodyPartMapStringToKey(pose_model, "LAnkle");

  bd.m_REye.x = keypoints[3*reye_id];
  bd.m_REye.y = keypoints[3*reye_id + 1];
  bd.m_REye.c = keypoints[3*reye_id + 2];

  bd.m_LEye.x = keypoints[3*leye_id];
  bd.m_LEye.y = keypoints[3*leye_id + 1];
  bd.m_LEye.c = keypoints[3*leye_id + 2];

  bd.m_REar.x = keypoints[3*rear_id];
  bd.m_REar.y = keypoints[3*rear_id + 1];
  bd.m_REar.c = keypoints[3*rear_id + 2];

  bd.m_LEar.x = keypoints[3*lear_id];
  bd.m_LEar.y = keypoints[3*lear_id + 1];
  bd.m_LEar.c = keypoints[3*lear_id + 2];

  bd.m_Nose.x = keypoints[3*nose_id];
  bd.m_Nose.y = keypoints[3*nose_id + 1];
  bd.m_Nose.c = keypoints[3*nose_id + 2];

  bd.m_Neck.x = keypoints[3*neck_id];
  bd.m_Neck.y = keypoints[3*neck_id + 1];
  bd.m_Neck.c = keypoints[3*neck_id + 2];

  bd.m_RShoulder.x = keypoints[3*rshoulder_id];
  bd.m_RShoulder.y = keypoints[3*rshoulder_id + 1];
  bd.m_RShoulder.c = keypoints[3*rshoulder_id + 2];

  bd.m_LShoulder.x = keypoints[3*lshoulder_id];
  bd.m_LShoulder.y = keypoints[3*lshoulder_id + 1];
  bd.m_LShoulder.c = keypoints[3*lshoulder_id + 2];

  bd.m_RElbow.x = keypoints[3*relbow_id];
  bd.m_RElbow.y = keypoints[3*relbow_id + 1];
  bd.m_RElbow.c = keypoints[3*relbow_id + 2];

  bd.m_LElbow.x = keypoints[3*lelbow_id];
  bd.m_LElbow.y = keypoints[3*lelbow_id + 1];
  bd.m_LElbow.c = keypoints[3*lelbow_id + 2];

  bd.m_RWrist.x = keypoints[3*rwrist_id];
  bd.m_RWrist.y = keypoints[3*rwrist_id + 1];
  bd.m_RWrist.c = keypoints[3*rwrist_id + 2];

  bd.m_LWrist.x = keypoints[3*lwrist_id];
  bd.m_LWrist.y = keypoints[3*lwrist_id + 1];
  bd.m_LWrist.c = keypoints[3*lwrist_id + 2];

  bd.m_RHip.x = keypoints[3*rhip_id];
  bd.m_RHip.y = keypoints[3*rhip_id + 1];
  bd.m_RHip.c = keypoints[3*rhip_id + 2];

  bd.m_LHip.x = keypoints[3*lhip_id];
  bd.m_LHip.y = keypoints[3*lhip_id + 1];
  bd.m_LHip.c = keypoints[3*lhip_id + 2];

  bd.m_RKnee.x = keypoints[3*rknee_id];
  bd.m_RKnee.y = keypoints[3*rknee_id + 1];
  bd.m_RKnee.c = keypoints[3*rknee_id + 2];

  bd.m_LKnee.x = keypoints[3*lknee_id];
  bd.m_LKnee.y = keypoints[3*lknee_id + 1];
  bd.m_LKnee.c = keypoints[3*lknee_id + 2];

  bd.m_RAnkle.x = keypoints[3*rankle_id];
  bd.m_RAnkle.y = keypoints[3*rankle_id + 1];
  bd.m_RAnkle.c = keypoints[3*rankle_id + 2];

  bd.m_LAnkle.x = keypoints[3*lankle_id];
  bd.m_LAnkle.y = keypoints[3*lankle_id + 1];
  bd.m_LAnkle.c = keypoints[3*lankle_id + 2];

  return bd;
}


void
correct_negative_coordinates(std::list<cvp_FaceDescriptor>& faces,
                             int width,
                             int height)
{
  typedef std::list<cvp_FaceDescriptor>::iterator IteratorType;
  for(IteratorType it=faces.begin() ; it!=faces.end() ; ++it)
    {
      if(it->m_FaceRegion.x<1)
        {
          it->m_FaceRegion.x = 1;
        }
      if(it->m_FaceRegion.y<1)
        {
          it->m_FaceRegion.y = 1;
        }
      if(it->m_FaceRegion.x + it->m_FaceRegion.width > width)
        {
          it->m_FaceRegion.width = width - it->m_FaceRegion.x - 2;
        }
      if(it->m_FaceRegion.y + it->m_FaceRegion.height > height)
        {
          it->m_FaceRegion.height = height - it->m_FaceRegion.y - 2;
        }
      if(it->m_FaceRegion.height<1)
        {
          it->m_FaceRegion.height = 1;
        }
      if(it->m_FaceRegion.width<1)
        {
          it->m_FaceRegion.width = 1;
        }
    }
}


} // namespace OpenCvPlus
