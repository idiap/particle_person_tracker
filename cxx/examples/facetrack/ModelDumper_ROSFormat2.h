// Copyright (c) 2018-2020 Idiap Research Institute
//
// ModelDumper_ROSFormat - class to dump main model data
//
// Authors: Olivier Canévet (olivier.canevet@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef _ModelDumper_ROSFormat2_h_
#define _ModelDumper_ROSFormat2_h_

#include <ros/ros.h>
#include "ModelDumper.h"
#include <tf/transform_broadcaster.h>

class ModelDumper_ROSFormat2: public ModelDumper
{
public:
  ModelDumper_ROSFormat2(const std::string& scope, const MainModel* model);
  virtual ~ModelDumper_ROSFormat2();
  virtual void update();

private:
  int m_DataIdx;
  ros::NodeHandle m_NodeHandle;
  ros::Publisher m_Publisher;
  // ros::Publisher m_posepub;
};

#endif /* _ModelDumper_ROSFormat2_h_ */
