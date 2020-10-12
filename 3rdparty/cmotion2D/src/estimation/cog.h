/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef cog_h
#define cog_h

int center_of_gravity(TImageShort *support, short label,
		      float *row_cog, float *col_cog, TWindow *win, int mode);

#endif
