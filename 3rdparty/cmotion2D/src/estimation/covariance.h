/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef covariance_h
#define covariance_h

bool update_covariance(double *m, Para *param);
void init_covariance(Para *param);
void print_covariance(Para *param);

#endif
