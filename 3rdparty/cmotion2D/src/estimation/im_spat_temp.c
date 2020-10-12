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

DESCRIPTION	:
Provides the spatio-temporal images management (displaced
gradients along x and y, and displaced frame difference DFD).

*/

// General include
#include "type.h"
#include "macro.h"
#include "interplt.h"
#include "im_spat_temp.h"
#include "para_mvt.h"

#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0

/*
INPUT		:
region        Address of the estimator support window.
label         Active label in the estimator support window.
im1		Address of an image 1 pyramid level.
im2		Address of an image 2 pyramid level.
gx		Address of a spatial gradient pyramid level for image 1 or 2.
gy		Address of a spatial gradient pyramid level for image 1 or 2.
win           Working window.
param		Motion model parameter given.

OUTPUT	:
igx		Address of the displaced image gradients pyramid level along
the horizontal axis:
igx = gx(pi + displacement, image 1 or 2)
igy		Address of the displaced image gradients pyramid level along
the vertical axis:
igy = gy(pi + displacement, image 1 or 2)
igt		Address of the DFD pyramid level:
igt = im2(pi+displ) - im1(pi)

DESCRIPTION	:
Compute the spatio-temporal images.

RETURN	:
The number of pixels considered during the computation. -1 if an error
occurs.

*/
int determine_im_spat_temp (const TImageShort *region, short label,
    const TImageShort *im1, const TImageShort *im2,
    const TImageFloat *gx, const TImageFloat *gy,
    TWindow win, Para *param, TImageFloat *igx,
    TImageFloat *igy, TImageFloat *igt)

{
  size_t	xsize = im1->nbco;	// Image size
  size_t	ysize = im1->nbli;	// Image size
  int	cnt   = 0;		// pixel counter

  double	coefs[MAXCOEFSMODEL];
  double	f[6];
  double	g[6];
  double	di;
  double  grx, gry, delta;
  int	x, y;			// Current pixel coordinates

  if (DEBUG_LEVEL2) {
    int i;
    printf("Model id before chg_repere: %d\n", param->id_model);
    for (i=0; i < MAXCOEFSMODEL; i ++)
      printf("%f ", param->thet[i]);
    printf("\n");
  }
  //
  // The motion model parameters are expressed with respect to the model
  // origin (generally the middle of the image). We transform it in the
  // image referential (0,0)
  //
  if (chgt_repere (param, coefs, 0, 0) == false)
    return -1;

  mvtcoef_to_poly (coefs, param->var_light, f, g, &di);

  if (DEBUG_LEVEL2) {
    int i;
    printf("Modele id after: %d\n", param->id_model);
    for (i=0; i < 6; i ++)
      printf("%f ", f[i]);
    printf("\n");
    for (i=0; i < 6; i ++)
      printf("%f ", g[i]);
    printf("\n");
  }
  if (DEBUG_LEVEL3)
    printf("determine_im_spat_temp() xsize: %d ysize: %d\n", (int)xsize, (int)ysize);


  // Displacement parameters, not motion
  f[1]++, g[2]++;

  if (model_degree(param->id_model)==2) {
    if (DEBUG_LEVEL2) printf("Degree = 2\n");
    for (y = win.dli; y < win.fli; y++) {
      size_t	off   = MIJ(0, y, 0, xsize);
      short	*preg = &region->ad[off];
      short	*pim1 = &im1->ad[off];
      float	*pigx = &igx->ad[off];
      float	*pigy = &igy->ad[off];
      float	*pigt = &igt->ad[off];

      double	y_ = (double) y;

      double	cf0 = f[0] + (y_ * ((y_ * f[5]) + f[2]));
      double	cf1 = f[1] + (y_ * f[4]);

      double	cg0 = g[0] + (y_ * ((y_ * g[5]) + g[2]));
      double	cg1 = g[1] + (y_ * g[4]);

      for (x = win.dco; x < win.fco; x++) {
        double	x_;
        double	xf, yf;

        if (preg[x] != label) {
          pigx[x] =
            pigy[x] =
            pigt[x] = (float) 0.0;
          continue;
        }

        x_ = (double) x;
        xf = cf0 + (x_ * ((x_ * f[3]) + cf1));
        yf = cg0 + (x_ * ((x_ * g[3]) + cg1));
        {
          INTERPLT2D_DECLARE(xsize, xf, yf);

          if (! INTERPLT2D_ISIN((int) xsize,
                (int) ysize)) {
            pigx[x] =
              pigy[x] =
              pigt[x] = 0.f;
            continue;
          }
          INTERPLT2D_PROCESS (float, gx->ad, grx);
          INTERPLT2D_PROCESS (float, gy->ad, gry);
          INTERPLT2D_PROCESS (short, im2->ad, delta);
          pigx[x] = (float) grx;
          pigy[x] = (float) gry;
          delta  += di;
          delta  -= (double) pim1[x];
          pigt[x] = (float) delta;
        }
        cnt++;
      }
    }
  }
  else if (model_degree(param->id_model)==1) {
    if (DEBUG_LEVEL2) printf("Degree = 1\n");
    for (y = win.dli; y < win.fli; y++) {
      size_t	off   = MIJ(0, y, 0, xsize);
      short	*preg = &region->ad[off];
      short	*pim1 = &im1->ad[off];
      float	*pigx = &igx->ad[off];
      float	*pigy = &igy->ad[off];
      float	*pigt = &igt->ad[off];

      double	y_ = (double) y;

      double	cf0 = f[0] + (y_ * f[2]);
      double	cg0 = g[0] + (y_ * g[2]);

      for (x = win.dco; x < win.fco; x++) {
        double	x_;
        double	xf, yf;

        if (preg[x] != label) {
          pigx[x] =
            pigy[x] =
            pigt[x] = (float) 0.0;
          continue;
        }

        x_ = (double) x;
        xf = cf0 + (x_ * f[1]);
        yf = cg0 + (x_ * g[1]);

        {
          INTERPLT2D_DECLARE(xsize, xf, yf);

          if (! INTERPLT2D_ISIN((int) xsize,
                (int) ysize)) {
            pigx[x] =
              pigy[x] =
              pigt[x] = (float) 0.0;
            continue;
          }
          INTERPLT2D_PROCESS (float, gx->ad, grx);
          INTERPLT2D_PROCESS (float, gy->ad, gry);
          INTERPLT2D_PROCESS (short, im2->ad, delta);
          pigx[x] = (float) grx;
          pigy[x] = (float) gry;
          delta  += di;
          delta  -= (double) pim1[x];
          pigt[x] = (float) delta;
        }

        cnt++;
      }
    }
  }
  else {
    if (DEBUG_LEVEL2) printf("Degree = 0\n");
    for (y = win.dli; y < win.fli; y++) {
      size_t	off   = MIJ(0, y, 0, xsize);
      short	*preg = &region->ad[off];
      short	*pim1 = &im1->ad[off];
      float	*pigx = &igx->ad[off];
      float	*pigy = &igy->ad[off];
      float	*pigt = &igt->ad[off];

      double	y_ = (double) y;
      double	cf0 = f[0];
      double	cg0 = g[0] + (y_ * g[2]);

      for (x = win.dco; x < win.fco; x++) {
        double	x_;
        double	xf, yf;

        if (preg[x] != label) {
          pigx[x] =
            pigy[x] =
            pigt[x] = (float) 0.0;
          continue;
        }

        x_ = (double) x;
        xf = cf0 + (x_ * f[1]);
        yf = cg0;

        
        INTERPLT2D_DECLARE(xsize, xf, yf);

        if (! INTERPLT2D_ISIN((int) xsize,
              (int) ysize)) {
          pigx[x] =
            pigy[x] =
            pigt[x] = (float) 0.0;
          continue;
        }
        INTERPLT2D_PROCESS (float, gx->ad, grx);
        INTERPLT2D_PROCESS (float, gy->ad, gry);
        INTERPLT2D_PROCESS (short, im2->ad, delta);
        pigx[x] = (float) grx;
        pigy[x] = (float) gry;
        delta  += di;
        delta  -= (double) pim1[x];
        pigt[x] = (float) delta;

        cnt++;
      }
    }
  }

  return (cnt);
}

