// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainWindow - class to visualise main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

// LOCAL INCLUDES
#include "MainModel.h"                                      // main model

/// @brief Configuration options for visualization
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

struct MainWindowConfig {
public:

};


/// @brief Class to visualise main model data
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class MainWindow
{

public:
  MainWindow(const MainModel& model);
  ~MainWindow();
  const IplImage *update();

private:
  void initialize();

  static void blend_rect(IplImage * image8u3,
                         const ImageProcessing::ip_RoiWindow& roi);

  void render_tracker(BICV::bicv_HeadPoseTracker * tracker,
                      IplImage * image8u3,
                      std::map<unsigned, CvScalar>& objId2ColourMap);
  void show_detected_faces(IplImage * image8u3);
  void show_skin_colour_likelihood(BICV::bicv_HeadPoseTracker * tracker,
                                   IplImage * image8u3,
                                   std::map<unsigned, CvScalar>& objId2ColourMap, float coeff);

  void show_general_skin_colour_classification(IplImage * src_image8u3,
                                               IplImage * dst_image8u3);
  void show_personal_skin_colour_classification(IplImage * src_image8u3,
                                                IplImage * dst_image8u3, BICV::bicv_HeadPoseTracker * tracker);
  void show_skin_colour_classification(IplImage * src_image8u3,
                                       IplImage * dst_image8u3,
                                       FaceColorModel::FaceColorModel * face_colour_model,
                                       const ImageProcessing::ip_RoiWindow& roi);

  void show_global_motion_support(IplImage * image8u3);
  void show_motion_support(BICV::bicv_HeadPoseTracker * tracker,
                           IplImage * image8u3,
                           std::map<unsigned, CvScalar>& objId2ColourMap, float coeff);
  void show_fps(IplImage * image8u3);


  bool mIsInitialized;
  const MainModel& mModel;
  IplImage * mImage8u3;
  IplImage * mImageBuf8u3;
#ifdef __STEREO_MATCHER_FOUND__
  IplImage * m_DepthImage64f1;
  IplImage * m_DisparityRenderedImage32f3;
  IplImage * m_DisparityRenderedImage8u3;
  IplImage * m_DisparityRenderedImage8u1;
#endif

};

#endif // __MAINWINDOW_H__
