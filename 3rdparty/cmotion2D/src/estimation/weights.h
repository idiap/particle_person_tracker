/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/


#ifndef weights_h
#define weights_h

void init_weights(TImageShort *region, int label, bool compute,
		  int robust_function, double C_value, TWindow win,
		  TImageFloat *imdt, TImageFloat *weights);
void update_weights(TImageFloat *imgx, TImageFloat *imgy, TImageFloat *imgt,
		    double *thet, EIdModel model, double li_c,
		    double co_c, TImageShort *zo, int label, TWindow win,
		    TImageFloat *im_pond, int robust_function, double C_value);

void compute_parameter_weights(double *pweights, EIdModel model,
			       bool var_light, TWindow win);

#endif	
