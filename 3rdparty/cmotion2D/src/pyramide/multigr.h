/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef multigr_h
#define multigr_h

bool Reduit_pyramide(TImageShort imag[], int &niveau_max,
		     double coef, int mem);

bool reduit_si (const TImageShort *ima_in, TImageShort *ima_out, double coef);

void ss_echant_pyramide(TImageShort *imag, int niv_max);
void ss_echantillonne(TImageShort *ima_in, TImageShort *ima_out);

#endif	/* multigr_h */
