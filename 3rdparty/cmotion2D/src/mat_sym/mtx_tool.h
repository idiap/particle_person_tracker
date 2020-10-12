/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
#ifndef __MTX_TOOL_H
#define __MTX_TOOL_H   



double	dasum (double *buf, size_t count, int incx);
int	daxpy (double *buf1, double *buf2, size_t count, double cst, int inc1, int inc2);
double	ddot(double *buf1, double *buf2, size_t count, int inc1, int inc2);
double	dotinc_double (const double *a, const double *b, size_t n,
                        int inca, int incb);
void	dot_mtx_rank (const double *a, size_t rownb, size_t colnb,
			double *dotrow, double *dotcol);
int	dscal (double *buf1, size_t count, double cst, int inc1);
int	dswap (double *buf1, double *buf2, size_t count, int inc1, int inc2);
int	idamax (double *buf, size_t count, int inc); 
void	show_mtx (double *buf, size_t raw, size_t col);


#endif	/* __MTX_TOOL_H	*/
