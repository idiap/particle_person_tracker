/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef pyramide_h
#define pyramide_h

#include <cmotion2d/Motion2D.h>

bool Etablit_pyr(TImageShort *ima, TImageFloat *gx, TImageFloat *gy,
		 int &nb_niv, double sigma_gauss, double coef_filt_avant_ss,
		 int taille_filtre_gradient, int mode, int mem);
void echange_pyr_shorti(TImageShort *ima1, TImageShort *ima2);
void echange_pyr_float(TImageFloat *ima1, TImageFloat *ima2);

#endif
