/**
 * @brief Various utility functions for OpenPose
 *
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef _CVP_OPENPOSEUTILS_H_
#define _CVP_OPENPOSEUTILS_H_

#include <list>

#include "opencvplus/cvp_FaceDescriptor.h"
#include "opencvplus/cvp_BodyDescriptor.h"

#include <openpose/core/headers.hpp>

namespace OpenCvPlus {

std::vector<double>
keypoints_to_bounding_box(const std::vector<float>& keypoints,
                          op::PoseModel pose_model,
                          int id);


cvp_BodyDescriptor
keypoints_to_body(const std::vector<float>& keypoints,
                  op::PoseModel pose_model,
                  int id);

void
correct_negative_coordinates(std::list<cvp_FaceDescriptor>& faces,
                             int width,
                             int height);

} // namespace OpenCvPlus

#endif
