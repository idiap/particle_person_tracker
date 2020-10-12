/**
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef _CVP_BODYDESCRIPTOR_H_
#define _CVP_BODYDESCRIPTOR_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include "cvp_JointDescriptor.h"

namespace OpenCvPlus {

struct cvp_BodyDescriptor
{
  cvp_JointDescriptor m_REye;
  cvp_JointDescriptor m_LEye;
  cvp_JointDescriptor m_REar;
  cvp_JointDescriptor m_LEar;
  cvp_JointDescriptor m_Nose;
  cvp_JointDescriptor m_Neck;
  cvp_JointDescriptor m_RShoulder;
  cvp_JointDescriptor m_LShoulder;
  cvp_JointDescriptor m_RElbow;
  cvp_JointDescriptor m_LElbow;
  cvp_JointDescriptor m_RWrist;
  cvp_JointDescriptor m_LWrist;
  cvp_JointDescriptor m_RHip;
  cvp_JointDescriptor m_LHip;
  cvp_JointDescriptor m_RKnee;
  cvp_JointDescriptor m_LKnee;
  cvp_JointDescriptor m_RAnkle;
  cvp_JointDescriptor m_LAnkle;

  /// Image width
  int m_ImageWidth;

  /// Image height
  int m_ImageHeight;

  /// Detection time
  boost::posix_time::ptime m_DetectionTime;

};

} // namespace OpenCvPlus



#endif
