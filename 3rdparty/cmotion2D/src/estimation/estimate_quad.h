/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
#ifndef estimate_quad_h
#define estimate_quad_h

bool estimate_QUA_PAN_DIV(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param);

bool estimate_QUA_PAN_TILT(TImageFloat *imgx, TImageFloat *imgy,
			   TImageFloat *imgt, TImageShort *zone_val,
			   int etiq, TWindow win,
			   TImageFloat *ima_pond, Para *d_param);

bool estimate_QUA_PAN_TILT_DIV(TImageFloat *imgx, TImageFloat *imgy,
			       TImageFloat *imgt, TImageShort *zone_val,
			       int etiq, TWindow win,
			       TImageFloat *ima_pond, Para *d_param);

bool estimate_QUA_2D(TImageFloat *imgx, TImageFloat *imgy,
		     TImageFloat *imgt, TImageShort *zone_val,
		     int etiq, TWindow win,
		     TImageFloat *ima_pond, Para *d_param);

bool estimate_QUA_COMPLET(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param);

#endif	
