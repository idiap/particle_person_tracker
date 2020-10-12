/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
#ifndef estimate_h
#define estimate_h

bool estimate(TImageFloat *imgx,TImageFloat *imgy, TImageFloat *imgt,
	      Para *d_param, TImageShort *zone_val, int etiq, TWindow win,
	      TImageFloat *ima_pond);

#endif
