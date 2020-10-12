/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

/*
 DESCRIPTION	: Compute the center of gravity position for a region in
		  an image.

*/

// General include
#include "type.h"
#include "macro.h"
#include "constant.h"

// Internal include
#include "cog.h"

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0


/*
  INPUTS        :
  support       Address of the support.
  label         Active label in the window.

  OUTPUTS       :
  row_cog       Center of gravity position along the vertical axis.
  col_cog       Center of gravity position along the horizontal axis.
 
  INPUT OR OUTPUT:
  win		Working window. Input if "mode=1", output if "mode=0".
		If mode=0, contains the windows geometical caracteristics.
 
  INPUT		:
  mode          Center of gravity mode computation.
                - If 1: "win" is considered as an input. Compute the COG
		  using points in "win" which are labeled "label" in the
		  "region" map.
                - if 0: All the points labeled "label" in "region" are
		  considered to compute the COG. Outputs the window
		  caracteristics.
 
  DESCRIPTION	:
  Compute the center of gravity of a a region.
 
*/

int center_of_gravity(TImageShort *support, short label,
		      float *row_cog, float *col_cog, TWindow *win, int mode)
{
  short       *preg;
  int         co,Sli=0,Sco=0,nbp=0;
  int         li,fli,fco;
  if (DEBUG_LEVEL1) printf("debut centre_de_gravite()\n");
  switch(mode) {
  case 0:
    if (DEBUG_LEVEL1) printf("mode 0\n");
    win->dco = support->nbco;
    win->fco = 0;
    win->dli = support->nbli;
    win->fli = support->nbli;
    preg = support->ad;
    fco=0; fli=0;
    for(li = 0; li < support->nbli; li++){
      win->fco = Max(fco,win->fco);
      for(co = 0; co < support->nbco; co++) {
	if(*preg++ == label) {
	  nbp++;
	  Sli += li; Sco += co;
	  fli=li; fco=co;
	  win->dli = Min(win->dli,li);
	  win->dco = Min(win->dco,co);
	}
      }
      if (DEBUG_LEVEL2) printf("windli:%d ", win->dli);
    }
    win->fco += 1; win->fli = fli+1;
    break;
    
  case 1:
  if (DEBUG_LEVEL1) printf("mode 1\n");
    for(li = win->dli; li < win->fli; li++) {
      preg = support->ad + li * support->nbco + win->dco;
      for(co = win->dco; co < win->fco; co++) {
	if(*preg++ == label) {
	  nbp++;
	  Sli += li;
	  Sco += co;
	}
      }
    }
    break;
  }
  
  if(nbp!=0) {
    *row_cog=(float)(Sli)/(float)(nbp);
    *col_cog=(float)(Sco)/(float)(nbp);
  }
  else {
    *row_cog=7.0;
    *col_cog=7.0;
  }

  if (DEBUG_LEVEL1) printf("fin centre_de_gravite()\n");
  return nbp;
}
