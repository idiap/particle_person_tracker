/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__UIAMEM_H
#define	__UIAMEM_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	copy_uint (const unsigned int *src, unsigned int *dst,
			size_t n);
extern	void	set_uint (unsigned int *buf, unsigned int v, size_t n);
extern  void	copyinc_uint (const unsigned int *src, unsigned int *dst,
			      size_t n, int incsrc, int incdst);

#endif	/* __UIAMEM_H	*/

