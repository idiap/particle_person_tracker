/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

/*
  
  Le fichier contient la gestion de la minimisation aux moidres
  carres ponderes du residuel ri =  X*delta_theta - Y.

*/

#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"
#include "daarith.h"
#include "damem.h"
#include "inverse_mat_sym.h"
#include "resoud_mat_sym.h"

#include "para_mvt.h"

#include "estimate.h"
#include "estimate_const.h"
#include "estimate_aff.h"
#include "estimate_quad.h"

// Definition of macro-instructions

//#define VERBOSE_MODEL

#define DEBUG_LEVEL1 0

/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  win          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of a motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate(TImageFloat *imgx,TImageFloat *imgy, TImageFloat *imgt,
	      Para *d_param, TImageShort *zone_val, int etiq, TWindow win,
	      TImageFloat *ima_pond)
{
  bool state = false;
  
  // Initialize the parameters to zero
  for (int i=0; i < MAXCOEFSMODEL; i++)
    d_param->thet[i] = 0.0;

  switch (d_param->id_model) {
  case MDL_TX:
    state = estimate_TX(imgx, imgt, zone_val,
			etiq, win, ima_pond, d_param);
    break;
    
  case MDL_TY:
    state = estimate_TY(imgy, imgt, zone_val,
			etiq, win, ima_pond, d_param);
    break;
    
  case MDL_TR:
    state = estimate_TR(imgx, imgy, imgt, zone_val,
			etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TX_DIV:
    state = estimate_AFF_TX_DIV(imgx, imgy, imgt, zone_val,
				etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TR_DIV:
    state = estimate_AFF_TR_DIV(imgx, imgy, imgt, zone_val,
				etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TR_ROT:
    state = estimate_AFF_TR_ROT(imgx, imgy, imgt, zone_val,
				etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TX_NULL:
    state = estimate_AFF_TX_NULL(imgx, imgy, imgt, zone_val,
				 etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TY_NULL:
    state = estimate_AFF_TY_NULL(imgx, imgy, imgt, zone_val,
				 etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_DIV_NULL:
    state = estimate_AFF_DIV_NULL(imgx, imgy, imgt, zone_val,
				  etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_ROT_NULL:
    state = estimate_AFF_ROT_NULL(imgx, imgy, imgt, zone_val,
				  etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_HYP1_NULL:
    state = estimate_AFF_HYP1_NULL(imgx, imgy, imgt, zone_val,
				   etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_HYP2_NULL:
    state = estimate_AFF_HYP2_NULL(imgx, imgy, imgt, zone_val,
				   etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_TR_ROT_DIV:
    state = estimate_AFF_TR_ROT_DIV(imgx, imgy, imgt, zone_val,
				    etiq, win, ima_pond, d_param);
    break;
  case MDL_AFF_COMPLET:
    state = estimate_AFF_COMPLET(imgx, imgy, imgt, zone_val,
				 etiq, win, ima_pond, d_param);
    break;
  case MDL_QUA_PAN_DIV:
    state = estimate_QUA_PAN_DIV(imgx, imgy, imgt, zone_val,
				 etiq, win, ima_pond, d_param);
    break;
  case MDL_QUA_PAN_TILT:
    state = estimate_QUA_PAN_TILT(imgx, imgy, imgt, zone_val,
				  etiq, win, ima_pond, d_param);
    break;
  case MDL_QUA_PAN_TILT_DIV:
    state = estimate_QUA_PAN_TILT_DIV(imgx, imgy, imgt, zone_val,
				      etiq, win, ima_pond, d_param);
    break;
  case MDL_QUA_2D:
    state = estimate_QUA_2D(imgx, imgy, imgt, zone_val,
			    etiq, win, ima_pond, d_param);
    break;
  case MDL_QUA_COMPLET:
    state = estimate_QUA_COMPLET(imgx, imgy, imgt, zone_val,
				 etiq, win, ima_pond, d_param);
    break;

  default:
    return false;
  }

#ifdef VERBOSE_MODEL
  printf("\nestimate() %d: ", d_param->nb_para);
  aff_Para(d_param);
#endif

  return state;
}
