/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__FAMEM_H
#define	__FAMEM_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	copy_float (const float *src, float *dst, size_t n);
extern	void	copyinc_float (const float *src, float *dst, size_t n,
		int incsrc, int incdst);
extern	void	set_float (float *buf, float v, size_t n);
extern	void	setinc_float (float *buf, float v, size_t n, int inc);


#endif	/* __FAMEM_H	*/

