/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

/*!
  \file CMotion2DPyramid.h
  \brief File to include to use CMotion2DPyramid.
*/

#ifndef CMotion2DPyramid_h
#define CMotion2DPyramid_h

//using namespace std;

#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif

#include "Motion2D.h"

using namespace std;

#if defined (WIN32)
#  if defined MOTION2D_DLL_EXPORTS
#     define MOTION2D_API __declspec( dllexport )
#  elif defined MOTION2D_DLL_IMPORTS
#     define MOTION2D_API __declspec( dllimport )
#  else
#     define MOTION2D_API
#  endif
#else
#     define MOTION2D_API
#endif

/*! \enum TPyramidError
  Error code values associated to the CMotion2DPyramid class.
*/
typedef enum {
  PYR_NO_ERROR = 0, /**< No error detected. */
  PYR_NOT_ALLOCATED = -1, /**<  Pyramid not allocated and not constructed. */
  PYR_LEVEL_NOT_ACCESSIBLE = -2, /**< Pyramid level not accessible. */
  PYR_TOO_MUCH_LEVELS = -3, /**< Too much levels to construct. */
  PYR_ALLOCATION_DONE = -4, /**< Memory pyramid is allocated yet. */
  PYR_NOT_ENOUGH_MEMORY = -5, /**< Not enough memory. */
  PYR_NOT_BUILD = -6 /**< Can not build the pyramid. */
} TPyramidError;

///////////////////////////////////////////////
// For spatio-temporal pyramid images
///////////////////////////////////////////////
class MOTION2D_API CMotion2DPyramid
{
 public:
  enum {
    NLEVELS_MAX = 10 /**< Maximal number of levels in a pyramid. */
  };

 public:
  CMotion2DPyramid();
  CMotion2DPyramid(const CMotion2DPyramid & pyramid);
  ~CMotion2DPyramid();

  TPyramidError allocate(int nrows, int ncols, int nlevels);
  TPyramidError build(const unsigned char *image);
  TPyramidError build(const unsigned char *image, int nrows, int ncols, int nlevels);
  TPyramidError build(const short *image);
  TPyramidError build(const short *image, int nrows, int ncols, int nlevels);
  void destroy();

  int getNumberOfLevels() const ;
  TPyramidError getNumberOfRows(int & rows,  int level=0) const ;
  TPyramidError getNumberOfCols(int & ncols, int level=0) const ;

  float *getRowsSpatialGradientDataAddress(TPyramidError &error, int level=0);
  float *getColsSpatialGradientDataAddress(TPyramidError &error, int level=0);
  short *getGaussianImageDataAddress(TPyramidError &error, int level=0);

  CMotion2DPyramid & exchange(CMotion2DPyramid & pyramid);

 private:
  void __panic(char *message);

 private:
  int nlevels; // Number of pyramid levels.
  int nrows;   // Number of rows for the floor level.
  int ncols;   // Number of cols for the floor level.

 public:
  // The maximum number of levels is given by NB_LEVELS_MAX
  TImageShort pyr_ima[NLEVELS_MAX]; // Temporal gradiens
  TImageFloat pyr_gx [NLEVELS_MAX]; // Spatial gradiens in x (cols)
  TImageFloat pyr_gy [NLEVELS_MAX]; // Spatial gradiens in y (rows)

  bool allocated; // Indicates if memory is allocated

};

#endif
