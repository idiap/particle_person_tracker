/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/


#ifndef __arithm_h
#define __arithm_h


#define	SIGN(x)		((x) < 0 ? -1 : 1)
#define	SWAP(a,b,c)	{(c) = (a); (a) = (b); (b) = (c);}


double	dsign (double a, double b);
int	dtrans (double *a, size_t raw, size_t col);


#endif	/* __arithm_h	*/
