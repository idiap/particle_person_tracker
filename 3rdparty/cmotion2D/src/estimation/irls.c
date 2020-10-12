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

 DESCRIPTION : Provides the iterated weight least-squares IRLS method.
  
*/

// General include
#include "type.h"
#include "macro.h"

// Local include
#include "irls.h"

// Internal include
#include "para_mvt.h"
#include "weights.h"
#include "estimate.h"

#define DEBUG_LEVEL1 0
//#define VERBOSE_MODEL

/*
  INPUTS      :
  region        Address of the estimator support window.
  label         Active label in the estimator support window.
  win           Size of the estimator support window.
 
  OUTPUT	:
  d_param	estimated motion model parameters.
 
  INPUTS      :
  compute_weights Select the initial weights computation. If false, all the
		weights are set to 1. If true, the weights are computed
		using the robust function.
  weights	Address of the array containing the weights.
  max_it_irls	Maximal number of iteration during the IRLS process.
  robust_function   Select the bi-weight function to use (if compute_weights is
		positionned at true).
                - If 0, Tukey biweight function, wi = ( 1 - (ri/C)2 )2
                - If 1, Talwar biweight function, si (ri/C)2 <= 1 wi = 0,
                                                  sinon           wi = 1
                - If 2, Cauchy biweight function, wi = 1 / ( 1 + (ri/C)2 )
                - If 3, Dennis Welsch biweight function, wi = exp( -(ri/C)2 ).
  C_value	Value of the C constant used in the robust function.
  imgx          Address of the spatial horizontal image gradients array.  
  imgy          Address of the spatial vertical image gradients array.
  imgt          Address of the DFD array: imgt = I(pi+depl,t+1) - I(pi,t).
  compute_covariance	If true compute the covariance matrix of the residual.
 
  OUTPUT       :
  covariance_last  Covariance matrix of the residual.
 
  DESCRIPTION :
  Compute the coefficients of the 2D polynomial motion model using either a
  least-mean square method (weights are set to 1), or an iterated weight least
  square method (IRLS). The process is stopped if a maximal number of
  itarations is reached, or if the incremental estimation is to small. In taht
  case the distance between 2 succesive estimations is less than 0.1 pixel.
   
*/

bool compute_irls(TImageShort *region, int label, TWindow win, Para *d_param,
		  bool compute_weights, TImageFloat *weights, int max_it_irls,
		  int robust_function, double C_value, TImageFloat *imgx,
		  TImageFloat *imgy, TImageFloat *imgt)
{
  double pweights[MAXCOEFSMODEL];
  Para   d_param_tmp;
  bool   state = false;

  if (DEBUG_LEVEL1) printf("Begin compute_irls()\n");
  
  // Save the previous motion model parameters
  copy_para(d_param, &d_param_tmp, CP_AND_INIT_THET);

  // Compute the weights associated to each model parameter in order the
  // compute a distance between 2 models using a linear combination of the
  // parameters
  compute_parameter_weights(pweights, d_param->id_model, d_param->var_light,
			    win);

  // Compute the weights associated to each pixel
  init_weights(region, label, compute_weights, robust_function,
	       C_value, win, imgt, weights);


#ifdef VERBOSE_MODEL
  printf("IRLS pweights ::\n");
  aff_theta(pweights);
  printf("max iter: %d\n", max_it_irls);
#endif

  int iter = 0;
  double d = 1.0; // Distance between 2 motion models

  while (iter < max_it_irls && d > 0.1)  {
    // Compute the modtion model parameters
    state = estimate(imgx, imgy, imgt, &d_param_tmp,
		     region, label, win, weights);
    if (state == false)
      break;

#ifdef VERBOSE_MODEL
    printf("Computed model %d at IRLS iteration %d\n",
	   d_param_tmp.id_model, iter);
    aff_Para(&d_param_tmp);
#endif

    // Update the weights for each pixel
    update_weights(imgx, imgy, imgt,
		   d_param_tmp.thet, d_param_tmp.id_model,
		   d_param_tmp.li_c, d_param_tmp.co_c,
		   region, label, win, weights, robust_function, C_value);
   
    // Compute the distance between the computed model and the previous one
    d = distance_para(*d_param, d_param_tmp, pweights, 0);
#ifdef VERBOSE_MODEL
    printf("distance: %f\n", d);
#endif

    // Update the new motion model
    copy_para(&d_param_tmp, d_param, CP_THET);

    iter++;
  }

#ifdef VERBOSE_MODEL
  if (state == true) {
    printf("Computed model %d after IRLS\n", d_param->id_model);
    aff_Para(d_param);
    printf("End compute_irls()\n");
  }
#endif
  if (DEBUG_LEVEL1) printf("End compute_irls()\n");
  return state;
}

