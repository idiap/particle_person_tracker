/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef variance_h
#define variance_h

double variance_im_fl(TImageShort *region, int etiq, TWindow *fen,
		      TImageFloat *imdift, int mode);

int rempli_tab_residuel(double *tab_res, TImageShort *zone, int etiqu,
			TWindow *fen, TImageFloat *imdift);
double medmed(double *tab_residuel, int nb_res);
void extrait_med_res(double *tab_residuel, int nb_res,
		     double *meilleure_mediane, int num_median);

double det_variance(double vi, double vf, int n_niv, int niv);

#endif
