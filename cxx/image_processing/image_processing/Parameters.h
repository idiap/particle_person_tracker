/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#define RESEARCH
//#define BASELINE
#define SIMPLE_REMOVE2  // only active if BASELINE is defined

#ifdef RESEARCH

//#define READ_ANNOTATION

#ifdef BASELINE
  #define SIMPLE_ADD
  #define SIMPLE_REMOVE
  //#define COLLECT_OBSERVATIONS
#else
  //#define SIMPLE_ADD
  //#define SIMPLE_REMOVE
  //#define SWITCHING_DYNAMICS
#endif
#else // !RESEARCH
#endif // RESEARCH



// enable/disable specific functionalities
#define MCMC
#define FULLPARTICLEFILTER
#define MRF_PROPOSAL   // use MRF + dynamics as proposal (instead of face det)
#define UPDATE_MODELS
//#define DENSE_MOTION_DYNAMICS
//#define INTERFACE_3D_COORDINATES


#ifdef RESEARCH
  //#define COLPOST_TRACKING
  //#define TEXTURE_BOUNDARY

  //#define TA2  // deprecated
  //#define SEGTEST
  //#define SEG_EDGE
  //#define SEG_CONTOUR
  #define MOTION  
  //#define CHANGE_DETECTION
  //#define VISUAL_ACTIVITY

  #define COLOUR_LIKELIHOOD
    //#define SHIRT2   // using one or several independent bounding boxes for shirt
    //#define POSE_LIKELIHOOD
    //#define CONTOUR_LIKELIHOOD
    //#define MOTION_CUE
    #define PERSON_MODELS
    //#define FD_TREE
    //#define FD_TREETEMPLATE
    //#define TREE_LIKELIHOOD	// not working yet
    #define DYNAMIC_PARTICLE_ALLOC_FOR_OBJECTS
    //#define SKIN_COLOUR_MODEL  // DEPRECATED: moved into tracking.conf
#else  // code for TA2
  //#define TA2  // deprecated
  //#define SEGTEST
  //#define SEG_EDGE
  //#define SEG_CONTOUR
  //#define MOTION  
  //#define CHANGE_DETECTION

  #define COLOUR_LIKELIHOOD
  //#define SHIRT2   // using one or several independent bounding boxes for shirt
  //#define POSE_LIKELIHOOD
  //#define CONTOUR_LIKELIHOOD
  //#define MOTION_CUE
  //#define FIX_ID_ORDER  // only used for review (fix ID 1 for left side and ID 2 for right side)
  #define PERSON_MODELS
#endif

//#define TORCHFD
//#define HEAD_EOH

#endif

