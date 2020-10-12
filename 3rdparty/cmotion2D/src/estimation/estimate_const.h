/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
#ifndef estimate_const_h
#define estimate_const_h





bool estimate_TX(TImageFloat *imgx, TImageFloat *imgt, 
		 TImageShort *zone_val, int etiq, TWindow win,
		 TImageFloat *ima_pond, Para *d_param);

bool estimate_TY(TImageFloat *imgy, TImageFloat *imgt, 
		 TImageShort *zone_val, int etiq, TWindow win,
		 TImageFloat *ima_pond, Para *d_param);

bool estimate_TR(TImageFloat *imgx, TImageFloat *imgy, TImageFloat *imgt, 
		 TImageShort *zone_val, int etiq, TWindow win,
		 TImageFloat *ima_pond, Para *d_param);



#endif	
