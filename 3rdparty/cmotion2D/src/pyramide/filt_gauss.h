/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef filt_gauss_h
#define filt_gauss_h


bool filtre_gauss(const TImageShort *ima_in, TImageShort *ima_out, const double *sigma2);
bool  calc_coef_filtre_gauss(FILTRE_GAUSS *filtre, const double *sigma2e);
void efface_memoire_filtre_gauss();

#endif	/* filt_gauss_h */
