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
  \file CMotion2DWarping.h
  \brief File to include to use CMotion2DWarping.
*/

#ifndef CMotion2DWarping_h
#define CMotion2DWarping_h

#include <stdio.h>
#include "Motion2D.h"
#include "CMotion2DModel.h"


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

//
// For image warping.
//
class MOTION2D_API CMotion2DWarping
{
 public:
  CMotion2DWarping();
  ~CMotion2DWarping();

  bool initBackWarp(int dst_nrows, int dst_ncols,
		    int dst_row_offset, int dst_col_offset);
  bool backWarp(unsigned char *src, int src_nrows, int src_ncols,
		unsigned char *dst, CMotion2DModel model);
  bool backWarp(short *src, int src_nrows, int src_ncols,
		short *dst, CMotion2DModel model);
  void getBackWarpPosition(int row, int col, float &d_row, float &d_col);
  bool warp(unsigned char *src, unsigned char *dst,
	    int nrows, int ncols, CMotion2DModel model);

  int   Nbli_ima_out;		// Nombre de lignes de l'image recalee.
  int   Nbcol_ima_out;		// Nombre de colonnes de l'image recalee.
  int   Offset_li;		// Decalage en lignes par rapport a l'origine.
  int   Offset_co;		// Decalage en colonn par rapport a l'origine.
  float Ivariation;		// Variation d'intensite entre l'image de
				// reference et l'image traitee.
  TImageFloat imFx;		// Deplacements en x a appliquer a chaque pixel
				// pour recaler une image.
  TImageFloat imFy;		// Deplacements en y a appliquer a chaque pixel
				// pour recaler une image.

};


#endif
