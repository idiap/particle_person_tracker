/**
 * Copyright (c) 2017-2020 Idiap Research Institute, Martigny, Switzerland
 * Written by Olivier Canévet <olivier.canevet@idiap.ch>
 *
 * See COPYING file for the complete license text.
 */

#ifndef _CVP_JOINTDESCRIPTOR_H_
#define _CVP_JOINTDESCRIPTOR_H_

namespace OpenCvPlus {

struct cvp_JointDescriptor
{
  double x;
  double y;
  double c; ///< Confidence

  cvp_JointDescriptor() { x=0; y=0; c=0; }

};

}



#endif
