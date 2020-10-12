/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef __INVERT_H
#define __INVERT_H


#include	<stdlib.h>

#include	"mtx_tool.h"
#include	"arithm.h"


int	dgeco (double *a, size_t lda, size_t n, int *ipvt,
		double *rcond, double *z);
int	dgedi (double *a, size_t lda, size_t n, int *ipvt,
		double *det, double *work, int job);
int	dgefa (double *a, size_t lda, size_t n, int *ipvt, int *info);


#endif	/* __INVERT_H	*/
