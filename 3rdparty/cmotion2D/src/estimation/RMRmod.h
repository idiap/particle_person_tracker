/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef RMRmod_h
#define RMRmod_h

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
	    int *ct_level_intro, int *lin_level_intro, int *quad_level_intr);
bool set_model_introduction_level(int mode, int level_ct, int level_lin,
				  int level_quad, int *ct_level_intro,
				  int *lin_level_intro, int *quad_level_intro);
int get_reliable_level(const TImageShort *pyr_support,
		       short label, int nb_level, int min_nb_pts,
		       TWindow win, int *nb_pts);
int count_nb_pts_region(const TImageShort *region, TWindow win, short label);

bool switch_to_constant(EIdModel id_model, bool var_light, Para *param);
bool switch_to_linear(EIdModel id_model, bool var_light, Para *param);
bool switch_to_quadratic(EIdModel id_model, bool var_light, Para *param);
void get_min_max_values(TImageFloat *imfl, TWindow win, TImageShort *region,
			short label, int mode, double *min, double *max);

#endif
