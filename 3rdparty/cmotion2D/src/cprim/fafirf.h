/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/



#ifndef	__FAFIRF_H
#define	__FAFIRF_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	firf_float (const float *src, float *dst, size_t n,
			const float *fir, size_t fsize);
extern	void	firfsym_float (const float *src, float *dst, size_t n,
			const float *fir, size_t fsize);
extern	void	firfsyminc_float (const float *src, float *dst, size_t n,
			int incsrc, int incdst, const float *fir, size_t fsize);


#endif	/* __FAFIRF_H	*/

