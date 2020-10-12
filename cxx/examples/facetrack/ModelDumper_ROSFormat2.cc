// Copyright (c) 2018-2020 Idiap Research Institute
//
// ModelDumper_ROSFormat - class to dump main model data
//
// Authors: Olivier Canévet (olivier.canevet@idiap.ch)
//
// See COPYING file for the complete license text.

#include <boost/foreach.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <stdlib.h>
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>
#include <vfoa/vfoa_CognitiveVfoaModel.h>

#include "ModelDumper_ROSFormat2.h"

#include "perception_msgs/PersonTracklet.h"
#include "perception_msgs/PersonTrackletArray.h"
#include "perception_msgs/TargetNameWithProbability.h"

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace BICV;
using namespace ImageProcessing;
using namespace OpenCvPlus;

static const string MODEL_DUMPER_NODE_NAME = "cpm_tracker";

ModelDumper_ROSFormat2::
ModelDumper_ROSFormat2(const std::string& scope,
                       const MainModel* model) :
        ModelDumper(scope, model), m_DataIdx(0)
{
  m_Publisher =
    m_NodeHandle.advertise<perception_msgs::PersonTrackletArray>(scope, 1);

  // m_posepub = m_NodeHandle.advertise<geometry_msgs::PoseStamped>("/zpose", 1);

  // m_posepub =
  //   m_NodeHandle.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

}

ModelDumper_ROSFormat2::
~ModelDumper_ROSFormat2()
{
}

void
ModelDumper_ROSFormat2::
update()
{
  typedef boost::posix_time::ptime ptime;

  const MainModel *main_model = model();
  ptime current_data_time = main_model->data_provider()->time();
  uint64_t ros_time = main_model->data_provider()->get_data_time_ns();

  const std::list<bicv_HeadPoseTracker*>& trackers =
    main_model->head_pose_trackers();

  const size_t nb_trackers = trackers.size();

  // std::cout << "main_model->data_provider()->image() " << main_model->data_provider()->image() << std::endl;

  // Return if no image yet. Should not happen, but just in case
  IplImage *image = main_model->data_provider()->image();

  if(!image) return;

  CvSize imageSize = cvGetSize(image);
  std::string tf_frame = main_model->data_provider()->get_tf_frame();

  perception_msgs::PersonTrackletArray msg;
  // msg.header.seq = m_DataIdx++;
  // msg.header.stamp = ros::Time::fromBoost(current_data_time);
  msg.header.stamp.fromNSec(ros_time);
  // msg.header.stamp = ros::Time::fromNSec(ros_time);
  msg.header.frame_id = tf_frame;
  msg.data.resize(nb_trackers);
  // std::cout << "trackers.size() " << trackers.size() << std::endl;

  // std::cout << "ros time ns_time_ns " << main_model->data_provider()->get_ros_time_ns() << std::endl;
  // std::cout << "msg.header.stamp.to_sec() " << msg.header.stamp.toNSec() << std::endl;
  // visualization_msgs::MarkerArray viz_arr;
  // viz_arr.markers.resize(nb_trackers);

  const std::list<cvp_BodyDescriptor>& bodies = main_model->bodies_cache();
  // std::cout << "bodies " << bodies.size() << std::endl;

  int idx = 0;
  BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers)
    {
      bicv_HeadPoseARTrackerState reported_state =
        mean(tracker->particle_distribution());
      ip_RoiWindow main_roi =
        HpParams2RoiConverter::hpparams2roi(reported_state.m_HeadPoseParamsCur);

      perception_msgs::PersonTracklet& tracklet = msg.data[idx];
      // visualization_msgs::Marker marker;

      // ID provided by tracker
      tracklet.tracklet_id = tracker->id();


      int body_idx = tracker->get_face_idx();
      // std::cout << "idx " << idx << " asso with detection " << body_idx << std::endl;

      // 3D position in meter (divided my 1000)
      CvScalar tracker_3dcoords = tracker->scene_coordinates_3D();

      double x = tracker_3dcoords.val[0]/1000.0;
      double y = tracker_3dcoords.val[1]/1000.0;
      double z = tracker_3dcoords.val[2]/1000.0;

      // Get head pose in radian
      const cvp_HeadPoseDiscreteDomain::HeadPose& head_pose =
        reported_state.m_HeadPoseParamsCur.m_HeadPose;

      double pan  = M_PI/180.0*head_pose.pan();
      double tilt = M_PI/180.0*head_pose.tilt();
      double roll = M_PI/180.0*head_pose.roll();

      tf::Quaternion q;
      q.setRPY(roll, tilt, pan);

      // std::cout << "[tracker] pan " << pan << " tilt " << tilt << " roll " << roll << std::endl;

      // Rotate pose in camera coordinate
      double theta = 90*M_PI/180;
      tf::Quaternion to_camera;
      to_camera.setRPY(theta, theta, 2*theta);

      // std::cout << "[tracker] q1 x " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

      q = to_camera*q;

      // std::cout << "[tracker] q2 x " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

      tracklet.head.position.x = x;
      tracklet.head.position.y = y;
      tracklet.head.position.z = z;
      tracklet.head.orientation.x = q.x();
      tracklet.head.orientation.y = q.y();
      tracklet.head.orientation.z = q.z();
      tracklet.head.orientation.w = q.w();

      tracklet.box.h = main_roi.m_iFirstRow;
      tracklet.box.w = main_roi.m_iFirstColumn;
      tracklet.box.height = main_roi.m_iHeight;
      tracklet.box.width = main_roi.m_iWidth;
      tracklet.box.image_height = imageSize.height;
      tracklet.box.image_width = imageSize.width;

      tracklet.joints.image_height = imageSize.height;
      tracklet.joints.image_width = imageSize.width;

      if(bodies.size()>0)
        {
          std::list<cvp_BodyDescriptor>::const_iterator it = bodies.begin();
          std::advance(it, body_idx);
          tracklet.joints.reye.h = it->m_REye.y;
          tracklet.joints.reye.w = it->m_REye.x;
          tracklet.joints.reye.c = it->m_REye.c;
          tracklet.joints.leye.h = it->m_LEye.y;
          tracklet.joints.leye.w = it->m_LEye.x;
          tracklet.joints.leye.c = it->m_LEye.c;
          tracklet.joints.rear.h = it->m_REar.y;
          tracklet.joints.rear.w = it->m_REar.x;
          tracklet.joints.rear.c = it->m_REar.c;
          tracklet.joints.lear.h = it->m_LEar.y;
          tracklet.joints.lear.w = it->m_LEar.x;
          tracklet.joints.lear.c = it->m_LEar.c;
          tracklet.joints.nose.h = it->m_Nose.y;
          tracklet.joints.nose.w = it->m_Nose.x;
          tracklet.joints.nose.c = it->m_Nose.c;
          tracklet.joints.neck.h = it->m_Neck.y;
          tracklet.joints.neck.w = it->m_Neck.x;
          tracklet.joints.neck.c = it->m_Neck.c;
          tracklet.joints.lshoulder.h = it->m_LShoulder.y;
          tracklet.joints.lshoulder.w = it->m_LShoulder.x;
          tracklet.joints.lshoulder.c = it->m_LShoulder.c;
          tracklet.joints.rshoulder.h = it->m_RShoulder.y;
          tracklet.joints.rshoulder.w = it->m_RShoulder.x;
          tracklet.joints.rshoulder.c = it->m_RShoulder.c;
          tracklet.joints.lelbow.h = it->m_LElbow.y;
          tracklet.joints.lelbow.w = it->m_LElbow.x;
          tracklet.joints.lelbow.c = it->m_LElbow.c;
          tracklet.joints.relbow.h = it->m_RElbow.y;
          tracklet.joints.relbow.w = it->m_RElbow.x;
          tracklet.joints.relbow.c = it->m_RElbow.c;
          tracklet.joints.lwrist.h = it->m_LWrist.y;
          tracklet.joints.lwrist.w = it->m_LWrist.x;
          tracklet.joints.lwrist.c = it->m_LWrist.c;
          tracklet.joints.rwrist.h = it->m_RWrist.y;
          tracklet.joints.rwrist.w = it->m_RWrist.x;
          tracklet.joints.rwrist.c = it->m_RWrist.c;
          tracklet.joints.lhip.h = it->m_LHip.y;
          tracklet.joints.lhip.w = it->m_LHip.x;
          tracklet.joints.lhip.c = it->m_LHip.c;
          tracklet.joints.rhip.h = it->m_RHip.y;
          tracklet.joints.rhip.w = it->m_RHip.x;
          tracklet.joints.rhip.c = it->m_RHip.c;
          tracklet.joints.lknee.h = it->m_LKnee.y;
          tracklet.joints.lknee.w = it->m_LKnee.x;
          tracklet.joints.lknee.c = it->m_LKnee.c;
          tracklet.joints.rknee.h = it->m_RKnee.y;
          tracklet.joints.rknee.w = it->m_RKnee.x;
          tracklet.joints.rknee.c = it->m_RKnee.c;
          tracklet.joints.lankle.h = it->m_LAnkle.y;
          tracklet.joints.lankle.w = it->m_LAnkle.x;
          tracklet.joints.lankle.c = it->m_LAnkle.c;
          tracklet.joints.rankle.h = it->m_RAnkle.y;
          tracklet.joints.rankle.w = it->m_RAnkle.x;
          tracklet.joints.rankle.c = it->m_RAnkle.c;
        }

#ifdef __VFOA_MODULE_FOUND__

        VfoaManager *vfoa_model = main_model->vfoa_model();
        distribution_t vfoa_distr =
                vfoa_model->GetVfoaDistributionForId(tracker->id());

        BOOST_FOREACH(const prob_element_t& elt, vfoa_distr)
          {
            std::ostringstream obj_name_oss;
            perception_msgs::TargetNameWithProbability target;
            obj_name_oss.str("");
            switch (elt.target.type)
              {
              case UNFOCUSSED_VALUE:
                obj_name_oss << "Unfocused";
                break;
              case OBJECT_INFO:
                obj_name_oss << vfoa_model->GetObjectInfo(elt.target.id).name;
                break;
              case PERSON_INFO:
                obj_name_oss << vfoa_model->GetPersonInfo(elt.target.id).name;
                break;
              default:
                obj_name_oss << "Error";
                break;
              }
            target.name = obj_name_oss.str().c_str();
            target.probability = elt.prob;
            tracklet.targets.push_back(target);
          }
#else
        VFOA::VfoaDistribution vfoaDistribution = main_model->vfoa_model()->
                compute_vfoa_distribution_Hellinger(tracker);
        VFOA::vfoa_CognitiveVfoaModelObjectInfo vfoaObjectInfo =
                mode(vfoaDistribution);
        head_object.vfoa_target = vfoaObjectInfo.mName.c_str();
#endif

      idx++;
    }

  // std::cout << "[ROSFormat2] Publish " << msg.header.stamp.toNSec() << std::endl;

  m_Publisher.publish(msg);
  // m_posepub.publish(viz_arr);

}



      // // {
      // //   tf::Transform transform;
      // //   transform.setOrigin(tf::Vector3(x,y,z));
      // //   tf::Quaternion q(0,0,0,1); // q.setRPY(0, 0, 0);
      // //   transform.setRotation(q);
      // //   m_br.sendTransform(tf::StampedTransform(transform,
      // //                                           ros::Time::fromBoost(current_data_time),
      // //                                           "camera_depth_optical_frame",
      // //                                           "person"));
      // // }


      // {
      //   tf::Transform transform;
      //   transform.setOrigin(tf::Vector3(x,y,z));
      //   // tf::Quaternion q(0.5,0.5,0.5,0.5); // q.setRPY(0, 0, 0);
      //   tf::Quaternion q(0.5,0.5,-0.5,0.5); // q.setRPY(0, 0, 0);
      //   // tf::Quaternion q(0,0,0,1); // q.setRPY(0, 0, 0);

      //   // double degx = 90/360.0*2*M_PI;;
      //   // double degy = /360.0*2*M_PI;;
      //   // double degz = /360.0*2*M_PI;;

      //   // q.setRPY(roll, tilt, pan);

      //   // q.setRPY(roll, tilt, pan);
      //   transform.setRotation(q);
      //   m_br.sendTransform(tf::StampedTransform(transform,
      //                                           ros::Time::fromBoost(current_data_time),
      //                                           "camera_depth_optical_frame",
      //                                           "person"));
      // }
