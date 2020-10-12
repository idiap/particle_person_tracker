/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef memoire_h
#define memoire_h

#include <cmotion2d/Motion2D.h>

bool Mem_pyramide(TImageShort *imag, int nbli, int nbco, int &niv_max);
bool Mem_pyramide_float(TImageFloat *imag, int nbli, int nbco, int & niv_max);
void free_p(TImageShort im[], int nb_niv);
void free_p_fl(TImageFloat *im, int nb_niv);

#endif
