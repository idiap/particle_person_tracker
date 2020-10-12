// Copyright (c) 2019-2020 Idiap Research Institute
//
// ModelDumperCs_CSVFormat - class to dump main model data in MOT challenge format
//
// Authors: Olivier Canévet (olivier.canevet@idiap.ch)
//
// See COPYING file for the complete license text.

#include "ModelDumper_MOTFormat.h"

#include "bayes_image/bicv_HeadPoseTracker.h"
#include "bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h"
#include "opencvplus/cvp_FaceDescriptor.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

using namespace BICV;
using namespace ImageProcessing;
using namespace OpenCvPlus;

ModelDumper_MOTFormat::
ModelDumper_MOTFormat(const std::string& dir_name,
                      const MainModel *model):
  ModelDumper(dir_name, model),
  m_face_id(0)
{
  if(!boost::filesystem::is_directory(dir_name) &&
     !boost::filesystem::create_directory(dir_name))
    {
      std::cout << name() << "Cannot make directory " << dir_name << std::endl;
    }

  boost::filesystem::path p(dir_name);

  boost::filesystem::path detection_file_name;
  detection_file_name = p / "detection.txt";

  boost::filesystem::path tracking_file_name;
  tracking_file_name = p / "tracking.txt";

  boost::filesystem::path vfoa_file_name;
  vfoa_file_name = p / "vfoa.txt";

  m_detection_file.open(detection_file_name.string().c_str());
  m_tracking_file.open(tracking_file_name.string().c_str());
  m_vfoa_file.open(vfoa_file_name.string().c_str());

  if(!m_detection_file.is_open())
    {
      std::cout << name() << " Detection is not open" << std::endl;
    }

  if(!m_tracking_file.is_open())
    {
      std::cout << name() << " Tracking is not open" << std::endl;
    }

  if(!m_vfoa_file.is_open())
    {
      std::cout << name() << " VFOA is not open" << std::endl;
    }
}


ModelDumper_MOTFormat::
~ModelDumper_MOTFormat()
{
  if(m_detection_file.is_open())
    {
      std::cout << name() << " Closing detection file" << std::endl;
      m_detection_file.close();
    }

  if(m_tracking_file.is_open())
    {
      std::cout << name() << " Closing tracking file" << std::endl;
      m_tracking_file.close();
    }

  if(m_vfoa_file.is_open())
    {
      std::cout << name() << " Closing VFOA file" << std::endl;
      m_vfoa_file.close();
    }
}


void
ModelDumper_MOTFormat::
update()
{
  if(!m_detection_file.is_open())
    {
      std::cout << name() << " Detection file is not open" << std::endl;
      return;
    }

  if(!m_tracking_file.is_open())
    {
      std::cout << name() << " Tracking file is not open" << std::endl;
      return;
    }

  if(!m_vfoa_file.is_open())
    {
      std::cout << name() << "VFOA file is not open" << std::endl;
      return;
    }

  const MainModel *main_model = model();
  IplImage *image = main_model->data_provider()->image();
  if(!image) return;
  CvSize imageSize = cvGetSize(image);


  const std::list<bicv_HeadPoseTracker*>& trackers =
    main_model->head_pose_trackers();

  const uint64_t image_id = static_cast<uint64_t>(main_model->data_provider()->image_id());
  const uint64_t data_time = static_cast<uint64_t>(main_model->data_provider()->get_data_time_ns());
  const uint64_t frame = (data_time > 0) ? data_time : image_id;

  const int original_width = main_model->data_provider()->get_original_width();
  const int original_height = main_model->data_provider()->get_original_height();

  // Rescale to original size
  const double ratio_width = static_cast<double>(original_width) / static_cast<double>(imageSize.width);
  const double ratio_height = static_cast<double>(original_height) / static_cast<double>(imageSize.height);

  // From https://motchallenge.net/instructions/
  //
  // <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
  BOOST_FOREACH(bicv_HeadPoseTracker *tracker, trackers)
    {
      bicv_HeadPoseARTrackerState reported_state =
        mean(tracker->particle_distribution());
      ip_RoiWindow main_roi =
        HpParams2RoiConverter::hpparams2roi(reported_state.m_HeadPoseParamsCur);

      const int id = tracker->id();

      const int bb_top = main_roi.m_iFirstRow;
      const int bb_left = main_roi.m_iFirstColumn;
      const int bb_height = main_roi.m_iHeight;
      const int bb_width = main_roi.m_iWidth;

      const double conf = 0.0;

      const cvp_HeadPoseDiscreteDomain::HeadPose& head_pose =
        reported_state.m_HeadPoseParamsCur.m_HeadPose;

      // In the MOT format, the last 3 elements are supposed to be the
      // 3D location of the face in the case 3D is used. Here, we use
      // roll, pitch, yaw in degrees.
      const double x = head_pose.roll();
      const double y = head_pose.tilt();
      const double z = head_pose.pan();

      m_tracking_file << frame << "," << id << ","
                      << bb_left*ratio_width << ","
                      << bb_top*ratio_height << ","
                      << bb_width*ratio_width << ","
                      << bb_height*ratio_height << ","
                      << conf << ","
                      << x << "," << y << "," << z
                      << std::endl;
    }

  const std::list<cvp_FaceDescriptor>& faces = main_model->faces_cache();
  std::list<cvp_FaceDescriptor>::const_iterator it;
  for(it=faces.begin() ; it!=faces.end() ; ++it)
    {
      const double conf = 0.0;

      const int id = m_face_id;
      m_face_id++;

      const int bb_top = it->m_FaceRegion.y;
      const int bb_left = it->m_FaceRegion.x;
      const int bb_height = it->m_FaceRegion.height;
      const int bb_width = it->m_FaceRegion.width;

      const int x = 0;
      const int y = 0;
      const int z = 0;

      m_detection_file << frame << "," << id << ","
                       << bb_left*ratio_width << ","
                       << bb_top*ratio_height << ","
                       << bb_width*ratio_width << ","
                       << bb_height*ratio_height << ","
                       << conf << ","
                       << x << "," << y << "," << z
                       << std::endl;
    }

  // VFOA
#ifdef __VFOA_MODULE_FOUND__
  const std::string sep = ",";
  const VfoaManager *vfoa_model = main_model->vfoa_model();

  BOOST_FOREACH(bicv_HeadPoseTracker *tracker, trackers)
    {
      const int id = tracker->id();

      distribution_t vfoa_distr =
        vfoa_model->GetVfoaDistributionForId(id);

      m_vfoa_file << frame << sep << id;

      BOOST_FOREACH(const prob_element_t& elt, vfoa_distr)
        {
        switch (elt.target.type)
          {
          case UNFOCUSSED_VALUE:
            m_vfoa_file << sep << "Unfocused" << sep << elt.prob;
            break;

          case OBJECT_INFO:
            m_vfoa_file << sep << vfoa_model->GetObjectInfo(elt.target.id).name
                        << sep << elt.prob;
            break;

          case PERSON_INFO:
            m_vfoa_file << sep << vfoa_model->GetPersonInfo(elt.target.id).name
                        << sep << elt.prob;
            break;

          default:
            m_vfoa_file << sep << "Error" << sep << "0";
            break;
          }
        }

      m_vfoa_file << std::endl;

    }
#endif

}
