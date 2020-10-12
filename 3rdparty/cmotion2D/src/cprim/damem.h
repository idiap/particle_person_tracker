/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__DAMEM_H
#define	__DAMEM_H


#include	<stdlib.h>	/* pour size_t	*/

extern	void	copy_double (register double *src, register double *dst,
			     size_t n);
extern	void	set_double (register double *buf, double v, size_t n);


#endif	/* __DAMEM_H	*/

