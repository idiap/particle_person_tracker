/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef	__ACAST_H
#define	__ACAST_H

#include	<stdlib.h>	/* pour size_t	*/
#include        <sys/types.h>

/* types a la POSIX	*/
typedef unsigned char	uchar;
#ifdef WIN32
typedef unsigned int	uint;
typedef unsigned long	ulong;
#endif // WIN32

extern	void	cast_double_float (const double *src, float *dst, size_t n);
extern	void	cast_float_double (const float *src, double *dst, size_t n);
extern	void	cast_float_short (const float *src, short *dst, size_t n);
extern	void	cast_short_float (const short *src, float *dst, size_t n);
extern	void	cast_short_uchar (const short *src, uchar *dst, size_t n);
extern	void	cast_uchar_short (const uchar *src, short *dst, size_t n);
extern	void	cast_uchar_uint (const uchar *src, uint *dst, size_t n);
extern	void	cast_char_short (const char *src, short *dst, size_t n);
extern	void	castinc_short_float (const short *src, float *dst, size_t n,
			int incsrc, int incdst);


#endif	/* __ACAST_H	*/

