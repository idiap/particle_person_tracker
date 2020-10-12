/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef im_spat_temp_h
#define im_spat_temp_h

int determine_im_spat_temp (const TImageShort *region, short label,
			    const TImageShort *im1, const TImageShort *im2,
			    const TImageFloat *gx, const TImageFloat *gy,
			    TWindow win, Para *param, TImageFloat *igx, 
			    TImageFloat *igy, TImageFloat *igt);


#endif	
