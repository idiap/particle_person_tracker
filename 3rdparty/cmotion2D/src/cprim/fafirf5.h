/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__FAFIRF5_H
#define	__FAFIRF5_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	firf5_float (const float *src, float *dst, size_t n,
			const float *fir);
extern	void	firf5inc_float (const float *src, float *dst, size_t n,
			int incsrc, int incdst, const float *fir);
extern	void	firf5sym_float (const float *src, float *dst, size_t n,
			const float *fir);
extern	void	firf5syminc_float (const float *src, float *dst, size_t n,
			int incsrc, int incdst, const float *fir);


#endif	/* __FAFIRF5_H	*/

