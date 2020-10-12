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

  DESCRIPTION	: Implement the RMRmod algorithm to estimate 2D parametric
                  motion models.

  See J.-M. Odobez, P. Bouthemy. Robust multiresolution estimation of
  parametric motion models. Journal of Visual Communication and Image
  Representation, 6(4):348-365, December 1995

*/


// General include
#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"  /* pour la macro MIJ */

// Internal include
#include "RMRmod.h"

// Local include
#include "irls.h"
#include "para_mvt.h"
#include "weights.h"
#include "im_spat_temp.h"
#include "variance.h"
#include "mem_est.h"
#include "../pyramide/multigr.h"


// Macro-instructions.
#define DEP_TOLERE	    0.1   // Deplacement minimum tolere. */

#define DEFAULT_CT_LEVEL    6     // default level for constant model
#define DEFAULT_LIN_LEVEL   2     // default level for affine model
#define DEFAULT_QUAD_LEVEL  1     // default level for quadratic model

#define VAR_INIT_MAX  100.0       // Maximal value of the C constant
#define VAR_INIT_MIN  25.0        // Minimal value of the C constant

//#define VERBOSE_MODEL
//#define VERBOSE_ESTIMATION

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0


/*
  INPUTS      :
  label         Active label in the estimator support window.
  win           Working window.
  level_max     Low resolution level in the pyramids
  ima1          Low-pass Gaussian pyramid on image 1.
  ima2          Low-pass Gaussian pyramid on image 2.
  gx1           Horizontal spatial gradients pyramid on image 1.
  gy1           Vertical spatial gradients pyramid on image 1.
  gx2           Horizontal spatial gradients pyramid on image 2.
  gy2           Vertical spatial gradients pyramid on image 2.
  max_it_irls   Maximal number of iterations in the IRLS method.
  robust_function   Select the bi-weight function to use:
                - If 0, Tukey biweight function
                - If 1, Talwar biweight function
                - If 2, Cauchy biweight function
                - If 3, Dennis Welsch biweight function.
  least_mean_square  Type of resolution:
		- If false, IRLS
		- If true, MRLS
  type_variance  Type of C constant computation:
                - If 0: The final value of C is given as input
                - If 1: Computed using the standard deviation,
                - If 2: robustly estimated.
  max_it_stab   Maximal number of iterations at a given pyramid level.


  OUTPUT        :
  paramet       Motion model parameter given at pyramid level 0.


  INPUTS      :
  mode_estim_init Indicate the manner of using possible initial model
		parameters:
                - If >= 10, all the parameters are directly estimated. Else,
		  estimation starts whith only the translational parameters.
		  See after the units quantity;
                - If >= 100, Use the given model origin for the estimation.
		  Else, the origin is fixed in the middle of the support
		  window. See after the units quantity;
                - Unit quantity:
                  Warning: If the modified equations are used, the initial
		           weights computation is based on the gradients on
			   image 2:
				Igx = gradIx(pi+depl, t+1)
				Igy = gradIy(pi+depl, t+1)
				Igt = I(pi+depl, t+1) - I(pi, t)
			   Otherwise, the gradients on image 1 are used:
	                        Igx = gradIx(pi, t)
                                Igy = gradIy(pi, t)
                                Igt = I(pi, t+1) - I(pi, t)

 		  . If 0: Starts the estimation from zero (no initial value
		          used) with no modified equations. The first
			  estimation is done with weights initialized to 1.
 		  . If 1: Starts the estimation using the given parameters as
		          input with no modified equations. The first
			  increment is estimated with the computed weights.
 		  . If 2: Same as 1, but use only the given translation to
			  initialize the estimator.
       	          . If 3: Use the modified equations. The first estimation
		          is computed with weights set to 1.
                  . If 4: Same as 3, but weights are computed.
 		  . If 5: Same as 3, but consider only modified equations for
		          the translation.
 		  . If 6: Same as 4, but consider only modified equations for
		          the translation.
 		  . If 7: Same as 4, but the M-estimator C constant is
		          selected weaker as of the departure (C=5).
           	  . If 8: Same as 6, but the M-estimator C constant is
		          selected weaker as of the departure (C=5).

  OUTPUT       :
  var            Variance used in the M-estimator weights computation (C
                 constant).

  INPUTS	:
  init_level_mode  Input levels computation mode:
                - 0: default input levels
                - 1: use the given input levelss.
  level_ct      Specified input level where constant models are considered.
  level_lin     Specified input level where linear models are considered.
  level_quad    Specified input level where quadratic models are considered.
  final_est_level  Pyramid level where the estimation is stopped.

  OUTPUT	:
  pyr_weights	Weights pyramid.

  INPUTS	:
  tx_pts_min    Reliable support rate for wich the estimation is considered
		as reliable.
  init_mem_estIsDone	  If true, the estimator initialization was done.

  OUTPUTS       :
  pyr_fl1	Displaced image gradients pyramid along the horizontal axis.
  pyr_fl2	Displaced image gradients pyramid along the vertical axis.
  pyr_fl3	DFD: I(pi+depl,t+1)-I(pi,t) pyramid .
  pyr_support	Estimator support pyramid.

  INPUTS	:
  compute_sigma2res	If true, compute the variance of the estimated
		parameters.
  verbose	Si 1, mode debug par affichage de printf().

  OUTPUTS	:
  ct_level_intro   Level where constant models were considered.
  lin_level_intro  Level where linear models were considered.
  quad_level_intro Level where quadratic models were considered.

  DESCRIPTION	:
  Compute the multi-resolution estimation of 2D parametric motion model
  by considering the pixels in the estimator support (pyr_support).

  Return true if the estimation process works, false otherwise.

*/
bool RMRmod(short label, const TWindow *win, int level_max,
	    const TImageShort ima1[], const TImageShort ima2[],
	    const TImageFloat gx1[], const TImageFloat gy1[],
	    const TImageFloat gx2[], const TImageFloat gy2[],
	    int max_it_irls, int robust_function, bool least_mean_square,
	    int type_variance, int max_it_stab, Para *paramet,
	    int mode_estim_init, double *var, int init_level_mode,
	    int level_ct,
	    int level_lin, int level_quad, int final_est_level,
	    TImageFloat pyr_weights[], double tx_pts_min,
	    TImageFloat *pyr_fl1, TImageFloat *pyr_fl2,
	    TImageFloat *pyr_fl3, TImageShort *pyr_support,
	    bool verbose,
	    int *ct_level_intro, int *lin_level_intro, int *quad_level_intro)
{
  int	i;
  int	level;	 // Current level in the multi-resolution framework
  int	scale;   // Scale used for subsampling values at a level.
  int	nb_pts_region;     // Number of points at a level.

  int	min_nb_pts_region;
  int	nb_level;
  int	iter_stab;
  bool	modified_equation = false; // Use of modified equations
  bool	estimate_all_para;       // All parameters are estimated from beginning
  double lic, coc;
  int   nb_par_or;
  bool   compute_weights = false;
  EIdModel id_model_or;
  bool var_light_or;

  double   d, pweights[MAXCOEFSMODEL];
  double var_init,var_finale = 1.0,vari,var_att,var_pred;
  TWindow  window;             // Current window

  // Level dependent variables
  Para	par_cum;           // Cumulate the model increments
  Para	delta_param;       // Model increment

  if (DEBUG_LEVEL3) printf("RMRmod() level_max: %d\n", level_max);
  // Compute the minimal number of points to be present at the first level
  min_nb_pts_region = 5 * paramet->nb_para;

  for (i = 1; i < level_max+1; i++)
    ss_echantillonne(&pyr_support[i - 1], &pyr_support[i]);

  // Save the model origin
  double _paramet_li_c = paramet->li_c;
  double _paramet_co_c = paramet->co_c;

  // If mode_estim_init > 100, compute the model in the given frame
  if (mode_estim_init >= 100) {
    lic = _paramet_li_c;
    coc = _paramet_co_c;
    mode_estim_init = mode_estim_init%100;
  }
  else { // The origin of the model is set to the middle of the window
    lic = (float)(win->dli + win->fli) / 2.f;
    coc = (float)(win->dco + win->fco) / 2.f;
  }

  lic = paramet->li_c;
  coc = paramet->co_c;

  nb_par_or = paramet->nb_para;
  id_model_or = paramet->id_model;
  var_light_or = paramet->var_light;

  // Determine the first level with enough points
  level = get_reliable_level(pyr_support, label, level_max+1,
			     min_nb_pts_region, *win, &nb_pts_region);

  // If not enough points, stop the process
  if (level == -1)
    return false;

  if(verbose)
    fprintf(stdout, "\n\nNumber of points founded at level %d : %d",
	    level,nb_pts_region);

  nb_level = level+1;

  // Determine levels where the estimation is started for intermediate models
  if (set_model_introduction_level(init_level_mode, level_ct,
				   level_lin, level_quad,
				   ct_level_intro, lin_level_intro,
				   quad_level_intro) == false)
    return false;

  level = Min(level, *ct_level_intro);

  // initialize the cumulated parameters to zero
  copy_para(paramet, &par_cum,CP_AND_INIT_THET);

  if (DEBUG_LEVEL2) {
    printf(" after init par_cum:\n");
    for(i=0;i<MAXCOEFSMODEL;i++)
      printf("%f ", par_cum.thet[i]);
    printf("\n");
  }

  // First motion model estimation at level: level
  if (level >= final_est_level)  {
    // Compute the window size for the estimation level
    det_fen(win, window, level, scale);

    // Initialize the parameters if necessary. par_cum is used to compute the
    // first spatio-temporal images. At least, par_cum contains the computed
    // multi-resolution motion model.

    // Are all the model parameters estimatied from the beginning
    if(mode_estim_init >= 10)
      estimate_all_para = true;
    else
      estimate_all_para = false;

    mode_estim_init = mode_estim_init%10;

    // Initialize par_cum. Note: par_cum contains the number of parameters to
    // estimate.
    switch(mode_estim_init)   {
    case 0:
      // Compute the spatio-temporal image using the first image
      modified_equation = false;

      // Initialize par_cum at zero
      copy_para(paramet,&par_cum, CP_AND_INIT_THET);

      par_cum.li_c = lic/scale;
      par_cum.co_c = coc/scale;
    break;
    case 1: case 2:
      // Compute the spatio-temporal image using the first image
      modified_equation = false;

      // Initialize par_cum at zero
      copy_para(paramet,&par_cum, CP_AND_INIT_THET);
      copy_para(paramet,&par_cum, CP_THET);

      change_level(&par_cum,&par_cum,level);

      break;

    case 3: case 4: case 7: case 5: case 6: case 8:
      // If the previous estimation is too bad, initialize the parameters at 0
      if (paramet->tx_pts <= tx_pts_min)  {
	modified_equation = false;

	copy_para(paramet,&par_cum, CP_AND_INIT_THET);

	par_cum.li_c = lic/scale;
	par_cum.co_c = coc/scale;
      }
      else {
	// Compute the spatio-temporal image using the second image
	modified_equation = true;

	change_level(paramet,paramet,level);

	// Copy the motion model (id, nb_para, var_light, origin, thet)
	copy_para(paramet,&par_cum, CP_MDL);

	par_cum.li_c = lic/scale;
	par_cum.co_c = coc/scale;
	if (chgt_repere(paramet,par_cum.thet,par_cum.li_c,par_cum.co_c)
	    == false) return false;
      }
      break;
    }

    switch(mode_estim_init)  {
    case 5: case 6: case 8:
      // We only keep the constant part of the model
      cut_para(&par_cum, MDL_TR);

      par_cum.nb_para = nb_par_or;
      par_cum.id_model = id_model_or;
      par_cum.var_light = var_light_or;

      break;
    }

    // Initialize the model increment: delta_param
    switch(mode_estim_init) {
    case 0: case 3: case 4: case 5: case 6: case 7: case 8:
      // Initialise delta_param.thet at 0
      copy_para(&par_cum, &delta_param, CP_AND_INIT_THET);
      break;

    case 1: case 2:
      change_level(paramet, paramet, level);

      // Copy paramet in delta_param
      copy_para(paramet,&delta_param, CP_MDL);

      delta_param.li_c = lic/scale;
      delta_param.co_c = coc/scale;
      if (chgt_repere(paramet,delta_param.thet,delta_param.li_c,
		      delta_param.co_c) == false) return false;

      break;
    }

    switch(mode_estim_init)  {
    case 2:
      // Initialize delta_param by keeping the constant part of the model
      cut_para(&delta_param, MDL_TR);
      break;
    }

    // Initialize the initial weights map
    if (least_mean_square)
      compute_weights = false;
    else  {
      switch(mode_estim_init)   {
      case 0 : case 3: case 5:
	compute_weights = false;   // Set the initial weights to 1.
	break;
      case 1: case 2: case 4: case 6: case 7: case 8:
	// If the previous estimation is too bad, set the weights to 1
	if (paramet->tx_pts <= tx_pts_min)
	  compute_weights = false;
	else
	  compute_weights = true; /* use a robust function: Tukey... */
	break;
      }
    }
    if (DEBUG_LEVEL2) {
      printf(" before determine_im_spat_temp() parcum:\n");
      for(i=0;i<MAXCOEFSMODEL;i++)
	printf("%f ", par_cum.thet[i]);
      printf("\n");
    }

    // Number of parameters to estimate: delta_param contains the number of
    // parameters for the current (intermediate or not) model to estimate and
    // par_cum contains the number of parameters for the final model.
    if(estimate_all_para == true) {
      delta_param.nb_para = nb_par_or;
      delta_param.id_model = id_model_or;
      delta_param.var_light = var_light_or;
    }
    else {
      // Set the initial model to estimate
      switch_to_constant(id_model_or,var_light_or,&delta_param);

      par_cum.id_model = delta_param.id_model;
      par_cum.nb_para = delta_param.nb_para;
      par_cum.var_light = delta_param.var_light;
    }

    if (DEBUG_LEVEL3)
      printf("RMRmod() call determine_im_spat_temp 1(level: %d)\n", level);
    // Compute the spatio-temporal images
    if (modified_equation == true) {
      if (determine_im_spat_temp(&pyr_support[level], label, &ima1[level],
				 &ima2[level], &gx2[level], &gy2[level],
				 window, &par_cum,
				 &pyr_fl1[level],&pyr_fl2[level],
				 &pyr_fl3[level]) == -1)
	return false;
    }
    else {
      if (determine_im_spat_temp(&pyr_support[level], label, &ima1[level],
				 &ima2[level], &gx1[level], &gy1[level],
				 window, &par_cum,
				 &pyr_fl1[level], &pyr_fl2[level],
				 &pyr_fl3[level]) == -1)
	return false;
    }

    // Compute the maximal temporal variation absolute value (mode 1) in orther
    // to set the C constant.
    get_min_max_values(&pyr_fl3[level], window, &pyr_support[level], label, 1,
		       &d, &var_init);

    var_init = Max(Min(VAR_INIT_MAX,var_init),VAR_INIT_MIN);

    // Eventual modification of the C constant (var_init) to take into account
    switch(mode_estim_init) {
    case 7:
      if(type_variance==0) {   // The C constant is given as input
	var_init = Max(*var, Min(var_init, 1.3* *var));
      }
      else  // We compute the C constant value to reach
	var_init = Max(10.0,var_init/2.0);

      break;
    case 8:
      if(type_variance==0) {   // The C constant is given as input
	var_init = Max(*var, Min(var_init, 1.7* *var));
      }
      else  // We compute the C constant value to reach
	var_init = Max(12.0,var_init/1.5);

      break;
    }

    // First increment motion model estimation
    vari = var_init;
    if(verbose) {
      fprintf(stderr,
	      "\nLevel %d : initial C constant  = %7.3f\n",level,var_init);
    }

    if (compute_irls(&pyr_support[level], label, window, &delta_param,
		     compute_weights, &pyr_weights[level],
		     max_it_irls, robust_function, vari, &pyr_fl1[level],
		     &pyr_fl2[level], &pyr_fl3[level]) == false)
      return false;

#ifdef VERBOSE_ESTIMATION
    printf("After first IRLS\ndelta\n");
    aff_Para(&delta_param);
#endif

    // Cumulate the increment model
    for (i=0;i< MAXCOEFSMODEL; i++)
      par_cum.thet[i] += delta_param.thet[i];

    par_cum.sigma2res = delta_param.sigma2res;

#ifdef VERBOSE_ESTIMATION
    printf("First estimation:\n");
    aff_Para(&par_cum);
#endif

    if (least_mean_square)
      compute_weights = false;
    else
      compute_weights = true;
    iter_stab = 0;


    while (iter_stab < 6)  {
#ifdef VERBOSE_MODEL
      printf("par_cum at while (iter_stab < 6) [iter: %d]\n",iter_stab);
      aff_Para(&par_cum);
#endif
      if (DEBUG_LEVEL3)
	printf("RMRmod() call determine_im_spat_temp 2(level: %d)\n", level);

      if (determine_im_spat_temp(&pyr_support[level], label, &ima1[level],
				 &ima2[level], &gx2[level], &gy2[level],
				 window, &par_cum,
				 &pyr_fl1[level], &pyr_fl2[level],
				 &pyr_fl3[level]) == -1)
	return false;

      var_init = Min(1.0,(0.70+iter_stab*0.025))*var_init;
      var_init = Max(var_init,10.0);
      vari = var_init;

      copy_para( &delta_param, &delta_param, CP_AND_INIT_THET);

      if (compute_irls(&pyr_support[level], label, window, &delta_param,
		       compute_weights, &pyr_weights[level], max_it_irls,
		       robust_function,vari, &pyr_fl1[level],
		       &pyr_fl2[level], &pyr_fl3[level]) == false)
	return false;

#ifdef VERBOSE_ESTIMATION
      printf("After IRLS 2 with iter_stab = %d\ndelta\n",iter_stab);
      aff_Para(&delta_param);
#endif

      // Add the increment model estimation
      for (i=0;i< MAXCOEFSMODEL; i++)
	par_cum.thet[i] += delta_param.thet[i];

      par_cum.sigma2res = delta_param.sigma2res;

#ifdef VERBOSE_ESTIMATION
      printf("translation estimation:\n");
      aff_Para(&par_cum);
#endif
      iter_stab++;
    }

    if(verbose) {
      fprintf(stdout,
	      "%d para estimated : level %d : C constant  = %7.3f\n",
	      delta_param.nb_para,level,var_init);
    }

    // Determine the final variance (C constant) to reach.
    switch(type_variance) {
    case 0:
      var_finale = *var; // The final variance is the input one
      break;

    case 1: // The C constant is computed classicaly
      if (DEBUG_LEVEL3)
	printf("RMRmod() call determine_im_spat_temp 3(level: %d)\n", level);
      if (determine_im_spat_temp(&pyr_support[level], label,&ima1[level],
				 &ima2[level],&gx2[level],&gy2[level], window,
				 &par_cum,&pyr_fl1[level],&pyr_fl2[level],
				 &pyr_fl3[level]) == -1)
	return false;
      var_finale = 2.5*variance_im_fl(&pyr_support[level],label,
				      &window,&pyr_fl3[level],0);
      break;

    case 2: // The C constant is computed robustly
      if (DEBUG_LEVEL3)
	printf("RMRmod() call determine_im_spat_temp 4(level: %d)\n", level);
      if (determine_im_spat_temp(&pyr_support[level], label, &ima1[level],
				 &ima2[level], &gx2[level], &gy2[level],
				 window, &par_cum,
				 &pyr_fl1[level], &pyr_fl2[level],
				 &pyr_fl3[level]) == -1)
	return false;
      // The 4.7 coefficient is recommended to ensure a better efficiency in
      // case of Gaussian noise.
      var_finale = 4.7*variance_im_fl(&pyr_support[level],label,
				      &window,&pyr_fl3[level],1);
      break;
    }



    while (level >= final_est_level)  {

#ifdef VERBOSE_MODEL
      printf("par_cum at while (level >= final_est_level)  [level: %d]\n",
	     level);
      aff_Para(&par_cum);
#endif

      det_fen(win,window,level,scale);
      var_att = det_variance(var_init,var_finale,nb_level,level);
      compute_parameter_weights(pweights, delta_param.id_model,
				delta_param.var_light, window);
      // Initialize the model origin for the increment
      delta_param.li_c = par_cum.li_c;
      delta_param.co_c = par_cum.co_c;

      iter_stab = 1;
      d = 1.0;
      var_pred=var_init;

      while (iter_stab <= max_it_stab && d > DEP_TOLERE / scale) {

#ifdef VERBOSE_MODEL
	printf("par_cum at while (iter_stab <= max_it_stab &&..) [iter_stab: %d]\n",iter_stab);
	aff_Para(&par_cum);
#endif

	vari = Min((var_init-var_att),var_att-var_finale);
	vari *= (double)(max_it_stab-iter_stab)/(double)(max_it_stab);
	vari += var_att;
	var_pred = Min(vari,var_pred);
	var_pred = vari;

	if (DEBUG_LEVEL3)
	  printf("RMRmod() call determine_im_spat_temp 5(level: %d)\n", level);
	if (determine_im_spat_temp(&pyr_support[level], label,&ima1[level],
				   &ima2[level],&gx2[level],&gy2[level],window,
				   &par_cum,&pyr_fl1[level],&pyr_fl2[level],
				   &pyr_fl3[level]) == -1)
	  return false;

	copy_para(&delta_param,&delta_param, CP_AND_INIT_THET);

	if (compute_irls(&pyr_support[level], label, window, &delta_param,
			 compute_weights, &pyr_weights[level], max_it_irls,
			 robust_function, vari, &pyr_fl1[level],
			 &pyr_fl2[level], &pyr_fl3[level]) == false)
	  return false;

#ifdef VERBOSE_ESTIMATION
	printf("After IRLS 3 with iter_stab = %d and level=%d\ndelta\n",
	       iter_stab,level);
	aff_Para(&delta_param);
#endif

	// Compute the distance between 2 models
	d = distance_para(delta_param, delta_param, pweights, 1);

	if (DEBUG_LEVEL2) printf("Distance para d: %f\n", d);

	// Add the model increments
	for (i=0;i< MAXCOEFSMODEL; i++)
	  par_cum.thet[i] += delta_param.thet[i];

	par_cum.sigma2res = delta_param.sigma2res;

#ifdef VERBOSE_ESTIMATION
	printf("Cumulated model:\n");
	aff_Para(&par_cum);
#endif

	iter_stab++;

	if (estimate_all_para == false) {
	  // If we use intermediate models, or if the intermediate motion model
	  // estimation converged

#ifdef VERBOSE_ESTIMATION
	  printf(" In if (estimate_all_para==0) ...before if(iter_stab==max_it_stab || d<=DEP_TOLERE/scale)\n");
	  printf(" iter_stab %d  max_it_stab %d  d %f  DEP_TOLERE/scale  %f \n",iter_stab,max_it_stab,d,DEP_TOLERE/scale);
#endif
	  if (iter_stab == max_it_stab || d <= DEP_TOLERE/scale) {

	    int degree_mdl_courant = model_degree(delta_param.id_model);
	    int degree_mdl_final   = model_degree(id_model_or);

	    // The final model is not yet estimated
	    if ( ( degree_mdl_courant < degree_mdl_final ) &&
		 // and the level to estimate linear models is reached
		 ( level <= *lin_level_intro ) &&
		 // and if the current model contains only translation
		 ( degree_mdl_courant == 0 ) &&
		 // and if an intermediate linear model exists
		 ( switch_to_linear(id_model_or,var_light_or,&delta_param)
		   == true)) {

	      iter_stab -=2;
	      d = 1.0;
	      par_cum.id_model = delta_param.id_model;
	      par_cum.nb_para = delta_param.nb_para;
	      par_cum.var_light = delta_param.var_light;

	      if(verbose)  {
		printf("Level %d--> 2 iterations with %d coefs\n",
		       level,delta_param.nb_para);
	      }
	      compute_parameter_weights(pweights, delta_param.id_model,
					delta_param.var_light, window);

	    }
	    else
	      // The final model is not yet estimated
	      if ( ( degree_mdl_courant < degree_mdl_final ) &&
		   // and if the level to estimate quadratic models is reached
		   ( level <= *quad_level_intro ) ) {

		switch_to_quadratic(id_model_or,var_light_or,&delta_param);

		iter_stab -=2;
		d = 1.0;
		par_cum.id_model = delta_param.id_model;
		par_cum.nb_para = delta_param.nb_para;
		par_cum.var_light = delta_param.var_light;

		if(verbose)
		{
		  printf("Level %d--> 2 iterations with %d coefs\n",
			 level,delta_param.nb_para);
		}

		compute_parameter_weights(pweights, delta_param.id_model,
					  delta_param.var_light, window);

	      }
	  }
	}
      }  // while(iter_stab <= max_it_stab && d > DEP_TOLERE / scale)/

      if(verbose) {
	printf("Level %d : final C constant  = %7.3f\n", level, vari);
      }
      if (level > final_est_level)
	change_level(&par_cum,&par_cum,-1);
      level--; // Next level

    } // while (level >= final_est_level)


    // Project the model to the pyramid level 0
    if (final_est_level > 0) {
      change_level(&par_cum, &par_cum, -(final_est_level));
    }
  } /* if level >=0 */

  for (i=0; i < MAXCOEFSMODEL; i ++)
    paramet->thet[i] = par_cum.thet[i];
  paramet->li_c = par_cum.li_c;
  paramet->co_c = par_cum.co_c;

#ifdef VERBOSE_ESTIMATION
  printf("RMRmod::RMRmod final model:\n");
  aff_Para(paramet);
#endif

  paramet->sigma2res = par_cum.sigma2res;
  memcpy(paramet->covariance, delta_param.covariance,
	 MAXCOEFSMODEL * MAXCOEFSMODEL * sizeof(double));

  if(verbose && paramet->compute_sigma2res)
    fprintf(stdout, "\nResidual variance: %7.3f \n", paramet->sigma2res);

  *var = var_finale;

  // Update the number of points used during the last estimation
  paramet->n_points=delta_param.n_points;
  paramet->tx_pts=delta_param.tx_pts;

  return (test_para(paramet->thet));
}



/*
  INPUTS        :
  mode		Input levels computation mode:
                - 0: default input levels
                - 1: use the given input levels
  level_ct      Specified input level where constant models are considered.
  level_lin     Specified input level where linear models are considered.
  level_quad    Specified input level where quadratic models are considered.

  OUTPUT       :
  ct_level_intro   Level where constant models are considered.
  lin_level_intro  Level where linear models are considered.
  quad_level_intro Level where quadratic models are considered.

  DESCRIPTION	:
  Determine the pyramid levels where the constant, affine or quadratic
  model are first considered.

*/

bool
set_model_introduction_level(int mode, int level_ct, int level_lin,
			     int level_quad, int *ct_level_intro,
			     int *lin_level_intro, int *quad_level_intro)
{

  switch(mode) {
  case 0:
    *ct_level_intro  = DEFAULT_CT_LEVEL;
    *lin_level_intro = DEFAULT_LIN_LEVEL;
    *quad_level_intro= DEFAULT_QUAD_LEVEL;
    break;
  case 1:
    *ct_level_intro  = level_ct;
    *lin_level_intro = Min(*ct_level_intro,  level_lin);
    *quad_level_intro= Min(*lin_level_intro, level_quad);
    break;
  default:
    fprintf(stderr, "Bad level determination for parameter introduction: %d\n", mode);
    return false;
  }
  return true;
}



/*
  INPUTS	:
  region        Address of the pyramid estimator support window.
  label         Active label in the estimator support window.
  nb_level	Number of levels in a pyramid.
  min_nb_pts	Minimal number of points to be present a la level.
  win           Size of the estimator support window.

  OUTPUT	:
  nb_pts        Number of points found at the reliable level.


  DESCRIPTION	:

  Return the reliable level in the pyramids where the number of points are
  sufficient for the estimation process. Return -1 if no reliable level is
  found.

*/
int get_reliable_level(const TImageShort *pyr_support, short label,
		       int nb_level, int min_nb_pts, TWindow win, int *nb_pts)
{
  int      level, scale, nb_pts_region;
  TWindow  window;

  if (DEBUG_LEVEL1) printf("Begin reliable_level_max()\n");
  if (DEBUG_LEVEL2) printf("min nbpoints : %d\n", min_nb_pts);
  if (DEBUG_LEVEL2) {
    printf("win: li %d %d co %d %d\n", win.dli, win.fli, win.dco, win.fco);
  }
  level = nb_level;
  nb_pts_region = 0;
  while (level > 0 && nb_pts_region < min_nb_pts)
  {
    level--;
    scale = UN << level;
    if (DEBUG_LEVEL2) printf("scale: %d\n", scale);
    window.dco = win.dco / scale + 1;
    window.fco = win.fco / scale;
    window.dli = win.dli / scale + 1;
    window.fli = win.fli / scale;
    nb_pts_region = count_nb_pts_region(&pyr_support[level], window, label);
    if (DEBUG_LEVEL2)printf("nb_pts_region: %d\n", nb_pts_region);
  }
  *nb_pts = nb_pts_region;

  if(nb_pts_region < min_nb_pts)
  {
    fprintf(stderr,"Error: Not enough points for motion estimation.\n");
    fprintf(stderr,"       The support is to small.\n");
    level=-1;
  }
  if (DEBUG_LEVEL2) printf("reliable level: %d\n", level);
  if (DEBUG_LEVEL1) printf("End reliable_level_max()\n");
  return level;
}



/*

  INPUTS      :
  region        Address of the estimator support window.
  label         Active label in the estimator support window.
  win           Size of the estimator support window.

  OUTPUT       : Aucune

  DESCRIPTION :
  Return the number of pixels in the estimation region which will
  be used during the estimation process.

*/
int count_nb_pts_region(const TImageShort *region, TWindow win, short label)
{
  int	        nb_points;
  int	        i,j;
  short         *p_im;

  nb_points = 0;

  for(j = win.dli;j<win.fli;j++) {
    p_im = ptrimage(region,j,win.dco);
    for(i = win.dco;i<win.fco;i++,p_im++) {
      if(*p_im == label)
	nb_points++;
    }
  }

  return nb_points;
}


/*
  INPUTS      :
  id_model      Motion model id to estimate.
  var_light     Indicates if the global illumination parameter is used.
  param         Characteristics of the model to estimate.

  OUTPUT       :
  param         Characteristics of the simplified model containing just
		constant parameters.

  DESCRIPTION	:

  Considering the final motion model id to estimate, determine the appropriate
  intermediate model containing just constant parameters to estimate first.

  Return false if no intermediate constant motion model exists, true otherwise.
*/

bool switch_to_constant(EIdModel id_model, bool var_light, Para *param)
{
  EIdModel id_model_const;

  switch (id_model) {
  case MDL_TX:
  case MDL_AFF_TX_DIV:
  case MDL_QUA_PAN_DIV:
  case MDL_AFF_TY_NULL:
    id_model_const = MDL_TX;
    break;
  case MDL_TY:
  case MDL_AFF_TX_NULL:
    id_model_const = MDL_TY;
    break;
  case MDL_QUA_PAN_TILT:
    id_model_const = MDL_QUA_PAN_TILT;
    break;
  default:
    id_model_const = MDL_TR;
    break;
  }

  param->id_model = id_model_const;

  // The global illumination parameter is only estimated for the final model
  if (id_model != id_model_const)
    param->var_light = false;
  else
    param->var_light = var_light;

  // Compute the number of coefficients for the intermediate model
  param->nb_para = Nb_Para_Modele(param->id_model);

  return true;
}


/*

  INPUTS	:
  id_model      Motion model id to estimate.
  var_light     Indicates if the global illumination parameter is used.
  param         Characteristics of the model to estimate.

  OUTPUT        :
  param         Characteristics of the simplified model containing just
		constant parameters.

  DESCRIPTION	:

  Considering the final motion model id to estimate, determine the appropriate
  intermediate model containing just constant and linear parameters to
  estimate.

  Return false if no intermediate linear motion model exists, true otherwise.
*/
bool switch_to_linear(EIdModel id_model, bool var_light, Para *param)
{

  EIdModel id_model_lin;

  switch (id_model) {
  case MDL_QUA_PAN_DIV:
    id_model_lin = MDL_AFF_TX_DIV;
    break;
  case MDL_QUA_PAN_TILT:
    // No intermediate linear motion model
    return false;
  case MDL_QUA_PAN_TILT_DIV:
    id_model_lin = MDL_AFF_TR_DIV;
    break;
  case MDL_QUA_2D:
  case MDL_QUA_COMPLET:
    id_model_lin = MDL_AFF_COMPLET;
    break;
  default:
    id_model_lin = id_model;  // For linear models it is the final one
    break;
  }

  param->id_model = id_model_lin;

  // The global illumination parameter is only estimated for the final model
  if (id_model != id_model_lin)
    param->var_light = false;
  else
    param->var_light = var_light;

  // Compute the number of coefficients for the intermediate model
  param->nb_para = Nb_Para_Modele(param->id_model);

  return true;
}

/*
  INPUTS	:
  id_model      Motion model id to estimate.
  var_light     Indicates if the global illumination parameter is used.
  param         Characteristics of the model to estimate.

  OUTPUT        :
  param         Characteristics of quadratic motion model parameters.

  DESCRIPTION :

  Considering the final motion model id, update the quadratic motion model to
  estimate.

  Return true.

*/
bool switch_to_quadratic(EIdModel id_model, bool var_light, Para *param)
{

  param->id_model = id_model;
  param->var_light = var_light;

  // Compute the number of coefficients for the intermediate model
  param->nb_para = Nb_Para_Modele(param->id_model);

  return true;
}




/*
  INPUTS	:
  input         Input address array.
  win           Size of the support window.
  region        Address of the support window array.
  label         Active label in the support window.
  mode		Computational mode:
		- if 0, consider the value on labeled pixels
		- if 1, consider the absolute value on labeled pixels
		- if 2, consider the value on all the window
		- if 3, consider the absolute value on all the window

  OUTPUTS      :
  min          Minimal value in the input array.
  max          Maximal value in the input array.

  DESCRIPTION	:
  Determine the minimal and maximal values in the input array.

*/

void get_min_max_values(TImageFloat *input, TWindow win, TImageShort *region,
			short label, int mode, double *min, double *max)
{
  int 	    li,co;
  short     *pregion;
  float	    *pinput;
  float	    mini,maxi;

  mini= 100000.0;
  maxi=-100000.0;
  switch(mode)
    {
    case 0:    // Consider the value on labeled pixels
      for(li=win.dli;li<win.fli;li++)
	{
	  pregion=ptrimage(region,li,win.dco);
	  pinput=ptrimage_float(input,li,win.dco);
	  for(co=win.dco;co<win.fco;co++,pregion++,pinput++)
	    if(*pregion==label)
	      {
		mini = Min(mini,*pinput);
		maxi = Max(maxi,*pinput);
	      }
	}
      break;

    case 1:    // Consider the absolute value on labeled pixels
      for(li=win.dli;li<win.fli;li++)
	{
	  pregion=ptrimage(region,li,win.dco);
	  pinput=ptrimage_float(input,li,win.dco);
	  for(co=win.dco;co<win.fco;co++,pregion++,pinput++)
	    if(*pregion==label)
	      {
		mini = (float) Min(mini,fabs(*pinput));
		maxi = (float) Max(maxi,fabs(*pinput));
	      }
	}
      break;

    case 2:    // Consider the value on each pixel on the window
      for(li=win.dli;li<win.fli;li++)
	{
	  pinput=ptrimage_float(input,li,win.dco);
	  for(co=win.dco;co<win.fco;co++,pinput++)
	    {
	      mini = Min(mini,*pinput);
	      maxi = Max(maxi,*pinput);
	    }
	}
      break;

    case 3:    // Consider the absolute value on each pixel on the window
      for(li=win.dli;li<win.fli;li++)
	{
	  pinput=ptrimage_float(input,li,win.dco);
	  for(co=win.dco;co<win.fco;co++,pinput++)
	    {
	      mini = Min(mini,*pinput);
	      maxi = Max(maxi,*pinput);
	    }
	}
      break;
    }
  *min=(double)(mini);
  *max=(double)(maxi);
}
