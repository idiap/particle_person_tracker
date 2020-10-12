// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumper_ROSFormat - class to dump main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                         // foreach loop
#include "boost/date_time/posix_time/posix_time.hpp" // boost posix time
#include <stdlib.h>


// PROJECT INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>  // parameters2roi converter
#include <vfoa/vfoa_CognitiveVfoaModel.h>

// LOCAL INCLUDES
#include "humavips_tracker/HeadObjects.h"
#include "ModelDumper_ROSFormat.h"                  // declaration of this

using namespace std;
using namespace BICV;
using namespace ImageProcessing;
using namespace OpenCvPlus;

////////////////////////// LOCAL DECLARATIONS ////////////////////////////////

static const string MODEL_DUMPER_NODE_NAME = "humavips_tracker";

/////////////////////////////// PUBLIC ///////////////////////////////////////

ModelDumper_ROSFormat::ModelDumper_ROSFormat(
        const std::string& scope, const MainModel* model) :
        ModelDumper(scope, model), m_DataIdx(0) {

    m_Publisher = m_NodeHandle.advertise<humavips_tracker::HeadObjects>(scope, 1);

} // ModelDumper_ROSFormat

/* virtual */ ModelDumper_ROSFormat::~ModelDumper_ROSFormat() {
} // ~ModelDumper_ROSFormat

/* virtual */ void ModelDumper_ROSFormat::update() {

    const MainModel * main_model = model();
    boost::posix_time::ptime current_data_time = main_model->data_provider()->time();
    const list<bicv_HeadPoseTracker*>& trackers = main_model->head_pose_trackers();

    CvSize imageSize = cvGetSize(main_model->data_provider()->image());

    // prepare head objects message of appropriate size
    humavips_tracker::HeadObjects head_objects;
    head_objects.header.seq = m_DataIdx++;
    head_objects.header.stamp = ros::Time::fromBoost(current_data_time);
    head_objects.head_objects.resize(trackers.size());

    int head_obj_idx = 0;
    BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers) {

        bicv_HeadPoseARTrackerState reported_state =
                mean(tracker->particle_distribution());
        ip_RoiWindow main_roi = HpParams2RoiConverter::hpparams2roi(
                reported_state.m_HeadPoseParamsCur);

        humavips_tracker::HeadObject& head_object = head_objects.head_objects[head_obj_idx++];

        // set id
        head_object.id.id = tracker->id();

        // set bounding box
        head_object.region.x = main_roi.m_iFirstColumn;
        head_object.region.y = main_roi.m_iFirstRow;
        head_object.region.width = main_roi.m_iWidth;
        head_object.region.height = main_roi.m_iHeight;
        head_object.region.image_width = imageSize.width;
        head_object.region.image_height = imageSize.height;

         // set reported head pose
        const cvp_HeadPoseDiscreteDomain::HeadPose& head_pose =
                reported_state.m_HeadPoseParamsCur.m_HeadPose;
        head_object.pose.x = head_pose.pan();
        head_object.pose.y = head_pose.tilt();
        head_object.pose.z = head_pose.roll();

        // set reported 3D scene coordinate
        CvScalar tracker_3dcoords = tracker->scene_coordinates_3D();
        head_object.position.x = tracker_3dcoords.val[0];
        head_object.position.y = tracker_3dcoords.val[1];
        head_object.position.z = tracker_3dcoords.val[2];

        // add VFOA specifications
#ifdef __VFOA_MODULE_FOUND__
        VfoaManager * vfoa_model = main_model->vfoa_model();
        distribution_t vfoa_distr =
                vfoa_model->GetVfoaDistributionForId(tracker->id());

        float proba;
        float max_proba = 0;
        ostringstream obj_name_oss;
        string obj_name;
        //TODO: implement mode computation for vfoa model distribution
        BOOST_FOREACH(const prob_element_t& elt, vfoa_distr) {
            obj_name_oss.str("");
            switch (elt.target.type) {
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
            proba = elt.prob;
            if (max_proba < proba) {
                obj_name = obj_name_oss.str();
                max_proba = proba;
            }
        }
        head_object.vfoa_target = obj_name.c_str();
#else
        VFOA::VfoaDistribution vfoaDistribution = main_model->vfoa_model()->
                compute_vfoa_distribution_Hellinger(tracker);
        VFOA::vfoa_CognitiveVfoaModelObjectInfo vfoaObjectInfo =
                mode(vfoaDistribution);
        head_object.vfoa_target = vfoaObjectInfo.mName.c_str();
#endif

        // add associated faces
        list<cvp_FaceDescriptor> current_face_detections =
                tracker->associated_face_detections_for_timestamp(
                current_data_time);
        head_object.faces.resize(current_face_detections.size());

        int face_idx = 0;
        BOOST_FOREACH(const cvp_FaceDescriptor& face_descriptor,
                    current_face_detections) {
            if (face_descriptor.m_Pose == CVP_FACEDETECTOR_FACE) {
                humavips_tracker::Face& face = head_object.faces[face_idx++];
                face.label = main_model->config().m_VariablesMap[
                    cvp_FaceDetectorFactory::OPT_FD_RSB_SCOPE_STR].as<string>();
                face.region.x = face_descriptor.m_FaceRegion.x;
                face.region.y = face_descriptor.m_FaceRegion.y;
                face.region.width = face_descriptor.m_FaceRegion.width;
                face.region.height = face_descriptor.m_FaceRegion.height;
                face.region.image_width = face_descriptor.m_ImageWidth;
                face.region.image_height = face_descriptor.m_ImageHeight;
                face.confidence = 1.0;
            }
        }

   }

   m_Publisher.publish(head_objects);

} // update
