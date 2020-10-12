/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__FAARITH_H
#define	__FAARITH_H


#include	<stdlib.h>	/* pour size_t	*/

#if	sun
#include	<values.h>
#endif

#if	MCC68K
#define	MINFLOAT	((float) 1.18e-38)
#endif

extern	void	add_float (float *a, const float *b, size_t n);
extern	void	addconst_float (float *a, float v, size_t n);
extern	void	addinc_float (float *a, const float *b, size_t n,
			int inca, int incb);
extern	void	multconst_float (float *a, float v, size_t n);
extern	void	multiply_float (float *a, const float *b, size_t n);
extern	void	subtract_float (register float *a, register float *b,
				size_t n);
extern	void	subinc_float (float *a, const float *b, size_t n,
			int inca, int incb);

#endif	/* __FAARITH_H	*/

