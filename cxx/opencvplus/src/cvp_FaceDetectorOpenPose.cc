/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_FaceDetectorOpenPose.h"
#include "opencvplus/cvp_OpenPoseUtils.h"
#include <opencv2/highgui/highgui.hpp>

namespace OpenCvPlus {

const std::string OPENPOSE_WINDOW = "OpenPose";

cvp_FaceDetectorOpenPose::
cvp_FaceDetectorOpenPose(const std::string& model_folder,
                         const std::string& pose_model,
                         const std::string& net_resolution,
                         int visu, int min_face_height):
  cvp_FaceDetector(CVP_FACEDETECTOR_FACE),
  m_visu(visu),
  m_min_face_height(min_face_height),
  m_is_initialised(0),
  m_pose_model(op::flagsToPoseModel(op::String(pose_model))),
  scaleAndSizeExtractor(op::flagsToPoint(op::String(net_resolution), "-1x128"),
                        0.0f, op::Point<int>(-1, -1), 1, 0.3)
{
  std::cout << "[FaceDetectorOpenPose]"
            << " Model " << pose_model
            << " resolution " << net_resolution
            << std::endl;

  std::cout << "[FaceDetectorOpenPose]"
            << " Folder " << model_folder
            << std::endl;

  if(m_visu>0)
    {
      cv::namedWindow(OPENPOSE_WINDOW, cv::WINDOW_NORMAL);
    }

  if(m_pose_model!=op::PoseModel::COCO_18 &&
     m_pose_model!=op::PoseModel::BODY_25)
    throw std::runtime_error("Only COCO_18 (COCO) and BODY_25 is handled");

  poseExtractorCaffe =
    std::make_shared<op::PoseExtractorCaffe>(m_pose_model, model_folder, 0);

  poseRenderer = std::make_shared<op::PoseGpuRenderer>
    (m_pose_model, poseExtractorCaffe, 0.05, true, 0.6, 0.7);

  poseRenderer->setElementToRender(0); // 0 is all parts

}


cvp_FaceDetectorOpenPose::
~cvp_FaceDetectorOpenPose()
{
}


void
cvp_FaceDetectorOpenPose::
prepare_internal(IplImage *image)
{
}


void
cvp_FaceDetectorOpenPose::
detect_faces_internal(IplImage * image,
                      const boost::posix_time::ptime& time,
                      std::list<cvp_FaceDescriptor>& storage)
{
  // Initialised here because otherwise, initialising in the
  // constructor may cause some thread issues (not allocated at the
  // right place). Still fuzzy, but initialising here is ok...
  if(!m_is_initialised)
    {
      poseExtractorCaffe->initializationOnThread();
      poseRenderer->initializationOnThread();
      m_is_initialised = 1;
    }

  std::vector<double> scaleInputToNetInputs;
  std::vector< op::Point<int> > netInputSizes;
  double scaleInputToOutput;
  op::Point<int> outputResolution;

  cv::Mat inputImage = cv::cvarrToMat(image);
  // cv::Mat inputImage = cv::cvarrToMat(image, true); // copy

  const op::Point<int> imageSize{inputImage.cols, inputImage.rows};

  std::tie(scaleInputToNetInputs,
           netInputSizes,
           scaleInputToOutput,
           outputResolution) = scaleAndSizeExtractor.extract(imageSize);

  const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(inputImage);

  const auto netInputArray = cvMatToOpInput.createArray(imageToProcess,
                                                        scaleInputToNetInputs,
                                                        netInputSizes);

  auto outputArray = cvMatToOpOutput.createArray(imageToProcess,
                                                 scaleInputToOutput,
                                                 outputResolution);

  poseExtractorCaffe->forwardPass(netInputArray,
                                  imageSize,
                                  scaleInputToNetInputs);

  const op::Array<float> poseKeypoints = poseExtractorCaffe->getPoseKeypoints();
  const std::vector<int> kpSize = poseKeypoints.getSize();
  const int n_parts = static_cast<int>(op::getPoseNumberBodyParts(m_pose_model));
  const int volume = static_cast<int>(poseKeypoints.getVolume());
  const int n_persons = poseKeypoints.getSize(0);

  if(volume != 3*n_persons*n_parts)
    {
      std::cout << "[FaceDetectorOpenPose] The array size is ";
      for(size_t i=0 ; i<kpSize.size() ; ++i)
        std::cout << kpSize[i] << " ";
      std::cout << " but volume is " << volume << std::endl;
      return;
    }

  std::vector<float> keypoints(volume, 0.0f);
  for(int i=0 ; i<volume ; ++i)
    keypoints[i] = poseKeypoints[i];

  // std::cout << "===========" << std::endl;
  // for(int i=0 ; i<n_persons ; ++i)
  //   {
  //     std::cout << "---" << std::endl;
  //     for(int j=0 ; j<3 ; ++j)
  //       {
  //         std::cout << keypoints[3*(i*n_parts + j)] << " ";
  //         std::cout << keypoints[3*(i*n_parts + j) + 1] << " ";
  //         std::cout << keypoints[3*(i*n_parts + j) + 2] << std::endl;
  //       }
  //   }

  if(m_visu > 0)
    {
      poseRenderer->renderPose(outputArray, poseKeypoints, scaleInputToOutput);
      // cv::Mat outputImage = opOutputToCvMat.formatToCvMat(outputArray);
      op::Matrix outputImage = opOutputToCvMat.formatToCvMat(outputArray);
      cv::Mat renderedImage = OP_OP2CVCONSTMAT(outputImage);
      cv::imshow(OPENPOSE_WINDOW, renderedImage);
    }

  //////////////////////////////////////////////////////////////////////
  // To big faces when using openface detector
  //////////////////////////////////////////////////////////////////////
  // std::vector< op::Rectangle<float> > faces =
  //   faceDetector.detectFaces(poseKeypoints, scaleInputToOutput);
  // cv::Mat display = inputImage.clone();
  // for(int i=0 ; i<faces.size() ; ++i)
  //   {
  //     cv::rectangle(display,
  //                   cv::Point(faces[i].x, faces[i].y),
  //                   cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
  //                   cv::Scalar(0,0,255), 3);
  //   }
  // cv::imshow("Face", display);
  //////////////////////////////////////////////////////////////////////

  std::list<cvp_FaceDescriptor> raw_faces;
  std::list<cvp_BodyDescriptor> raw_bodies;

  for(int i=0 ; i<n_persons ; ++i)
    {
      std::vector<double> bb = keypoints_to_bounding_box(keypoints,
                                                         m_pose_model,
                                                         i);

      if(bb[0]==0 && bb[1]==0 && bb[2]==0 && bb[3]==0) continue;

      cvp_FaceDescriptor temp_fd;
      temp_fd.m_ImageWidth = image->width;
      temp_fd.m_ImageHeight = image->height;

      temp_fd.m_FaceRegion.x = bb[0];
      temp_fd.m_FaceRegion.y = bb[1];
      temp_fd.m_FaceRegion.width  = bb[2];
      temp_fd.m_FaceRegion.height = bb[3];

      temp_fd.m_DetectionTime = time;
      temp_fd.m_Pose = pose();

      // Skip small faces
      if(bb[2]>=m_min_face_height && bb[3]>=m_min_face_height)
        {
          raw_faces.push_back(temp_fd);
          cvp_BodyDescriptor bd = keypoints_to_body(keypoints,
                                                    m_pose_model,
                                                    i);
          raw_bodies.push_back(bd);
        }
    }

  correct_negative_coordinates(raw_faces, image->width, image->height);

  std::copy(raw_faces.begin(), raw_faces.end(), back_inserter(storage));
  std::copy(raw_bodies.begin(), raw_bodies.end(), back_inserter(mBodies));
}

} // namespace OpenCvPlus
