/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef irls_h
#define irls_h

bool compute_irls(TImageShort *region, int label, TWindow win, Para *d_param,
		  bool compute_weights, TImageFloat *weights, int max_it_irls,
		  int robust_function, double C_value,
		  TImageFloat *imgx, TImageFloat *imgy, TImageFloat *imgt);

#endif
