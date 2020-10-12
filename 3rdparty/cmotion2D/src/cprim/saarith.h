/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__SAARITH_H
#define	__SAARITH_H


#include	<stdlib.h>	/* pour size_t	*/


extern	void	abs_short (short *a, size_t n);
extern	void	add_short (short *a, const short *b, size_t n);
extern	void	divconst_short (short *a, short d, size_t n);
extern	void	binary_short (short *a, size_t n, short low, short high,
			short in, short out);
extern	void	histogram_short (const short *a, size_t n, long *lut);
extern	void	lut_short (short *a, size_t n, const short *lut);
extern	int	minmax_short (const short *a, size_t n,
			size_t *minp, size_t *maxp);
extern	void	multiply_short (short *a, const short *b, size_t n);
extern	void	negpos_short (const short *a, size_t n,
			long *negp, long *posp);
extern	void	subtract_short (short *a, const short *b, size_t n);

#endif	/* __SAARITH_H	*/

