/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__UCAMEM_H
#define	__UCAMEM_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	copy_uchar (const unsigned char *src, unsigned char *dst,
			    size_t n);
extern	void	copyinc_uchar (const unsigned char *src, unsigned char *dst,
			       size_t n, int incsrc, int incdst);
extern	void	set_uchar (unsigned char *buf, unsigned char v, size_t n);

#endif	/* __UCAMEM_H	*/

