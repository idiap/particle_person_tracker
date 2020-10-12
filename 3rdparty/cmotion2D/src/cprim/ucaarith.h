/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__UCAARITH_H
#define	__UCAARITH_H


#include	<stdlib.h>	/* pour size_t	*/
#include        <sys/types.h>

typedef unsigned char	uchar;

extern	void	add_uchar (uchar *a, const uchar *b, size_t n);
extern	void	subtract_uchar (uchar *a, const uchar *b, size_t n);

#endif	/* __UCAARITH_H	*/

