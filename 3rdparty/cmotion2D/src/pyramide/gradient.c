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
  
 DESCRIPTION	: Le fichier contient la gestion de la construction
                  des pyramides de gradients temporels.

*/


/* Inclusion des fichiers generaux.			*/
#include "type.h"
#include "macro.h"
#include "interplt.h"

/* Inclusion des prototypes des procedures internes.	*/
#include "gradient.h"

#include "acast.h"
#include "famem.h"
#include "faarith.h"
#include "fafirf3.h"
#include "fafirf5.h"
#include "fafirf7.h"


/* Declaration des procedures locales.			*/
static	bool	firx_n_float (TImageShort *ima_in, TImageFloat *ima_filt,
			      float *coefs, size_t taille);
static	bool	firy_n_float (TImageShort *ima_in, TImageFloat *ima_filt,
			      float *coefs, size_t taille);

/*
 * PROCEDURE	: Derivees_float
 *
 * INPUT       :
 * src		  Pointeur sur un niveau de la pyramide de l'image.
 *
 * OUTPUTS      :
 * dst_gx	  Pointeur sur un niveau la pyramide de gradient suivant x.
 * dst_gy	  Pointeur sur un niveau la pyramide de gradient suivant y.
 *
 * INPUT       :
 * taille         Taille du filtre pour le calcul des gradients.
 *                Les taillees possibles sont: 3, 7 ou 9.
 * PRINTF_ON	  Si 1, mode debug par affichage de printf().
 *
 * DESCRIPTION	:
 * La procedure construit les gradients spatiaux, suivant x et y d'une image
 * "src". Le calcul des gradients ne se fait qu'a un seul niveau de la
 * pyramide. La methode utilisee est celle decrite dans l'article de
 * Vieville et Faugeras.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Derivees_float (TImageShort *src, TImageFloat *dst_gx,
		     TImageFloat *dst_gy, int taille)
{

#define	XFSIZE	3
#define	YFSIZE	3
  /* filtre 3 x 3 de detection de gradients horizontaux	*/
  static	short	firx_33[YFSIZE][XFSIZE] = {
    {-1,	-2,	-1},
    {0,	0,	0},
    {1,	2,	1}
  };
#undef	XFSIZE
#undef	YFSIZE

#define	XFSIZE	5
#define	YFSIZE	5
  /* filtre 5 x 5 de detection de gradients horizontaux	*/
  static	short	firx_55[YFSIZE][XFSIZE] = {
    {135,	-15,	-65,	-15,	135},
    {-188,	-263,	-288,	-263,	-188},
    {0,	0,	0,	0,	0},
    {188,	263,	288,	263,	188},
    {-135,	15,	65,	15,	-135}
  };
#undef	XFSIZE
#undef	YFSIZE

#define	XFSIZE	7
#define	YFSIZE	7
  /* filtre 7 x 7 de detection de gradients horizontaux	*/
  static	short	firx_77[YFSIZE][XFSIZE] = {
    {1115,	380,	-61,	-208,	-61,	380,	1115},
    {-610,	-1100,	-1394,	-1492,	-1394,	-1100,	-610},
    {-711,	-956,	-1103,	-1152,	-1103,	-956,	-711},
    {0,	0,	0,	0,	0,	0,	0},
    {711,	956,	1103,	1152,	1103,	956,	711},
    {610,	1100,	1394,	1492,	1394,	1100,	610},
    {-1115,	-380,	61,	208,	61,	-380,	-1115}
  };
#undef	XFSIZE
#undef	YFSIZE

#define	XFMAXSIZE	7
#define	YFMAXSIZE	7

  static	float	fir[YFMAXSIZE * XFMAXSIZE];

  short	*firx;	/* pointeur sur le filtre entier	*/
  size_t	fdiv;	/* diviseur du filtre			*/
  size_t	fsize;	/* taille du filtre			*/

  switch (taille) {
  case 3 :
    firx = &firx_33[0][0];
    fdiv = 8;
    break;
  case 5 :
    firx = &firx_55[0][0];
    fdiv = 1680;
    break;
  case 7 :
    firx = &firx_77[0][0];
    fdiv = 28224;
    break;
  default :
    printf("\nMauvaise taille du filtre.\n");
    return false;
  }

  fsize = (size_t) (taille * taille);

  cast_short_float (firx, fir, fsize);
  multconst_float (fir, 1.F / (float) fdiv, fsize);

  bool state;
  
  state = firx_n_float (src, dst_gx, fir, (size_t) taille);
  if (state == false)
    return false;
  
  state = firy_n_float (src, dst_gy, fir, (size_t) taille);
  return state;
}



/*
 * PROCEDURE	: firx_n_float
 *
 * INPUT       :
 * ima_in         Pointeur sur la pyramide de l'image.
 *
 * OUTPUT       :
 * ima_filt       Pointeur sur la pyramide de gradient spatial.
 *                On obtient les gradients suivant x ou y en fonction de
 *                la valeur des coefficients du filtre contenus dans "coefs".
 *
 * INPUTS      :
 * coefs          Pointeur sur les coefficients du filtre.
 * taille	 Taille du filtre.
 *
 * DESCRIPTION	:
 * La procedure effectue un filtrage taille X taille de
 * l'image "ima_in". Ce filtrage correspond au calcul d'un gradients,
 * suivant x ou y, selon la valeur des coefficients du filtre contenus
 * dans "coefs"
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
static	bool	firx_n_float (TImageShort *ima_in, TImageFloat *ima_filt,
			      float *coefs, size_t taille)
{
  size_t	midsize  = taille / 2;
  size_t	xsize	 = (size_t) ima_in->nbco;
  size_t	ysize	 = (size_t) ima_in->nbli;
  void		(*fir_float) (const float *, float *, size_t,  const float *);
  float		*src;
  float		*dst;
  size_t	x;

  // Test si filtre applicable
  if ((xsize < midsize) || (ysize < midsize)) {
    return false;
  }
  
  switch (taille) {
  case 3	: fir_float = firf3sym_float; break;
  case 5	: fir_float = firf5sym_float; break;
  case 7	: fir_float = firf7sym_float; break;
  default : return false;
  }

  if ((src = (float *) malloc (ysize * sizeof(float))) == (float *) NULL)
    return false;

  if ((dst = (float *) malloc (ysize * sizeof(float))) == (float *) NULL) {
    free ((void *) src);
    return false;
  }

  set_float (ima_filt->ad, 0.F, xsize * ysize);

  for (x = 0; x < xsize; x++) {
    int	i;

    castinc_short_float (MIJ(ima_in->ad, 0, x, xsize), src, ysize,
			 (int) xsize, 1);

    for (i = 0; i < (int) midsize; i++) {
      (*fir_float) (src, dst, ysize - (taille - 1),
		    MIJ(coefs, i, 0, taille));

      if ((x + midsize >= (size_t) i) && (x + midsize < xsize + i))
	addinc_float (
		      MIJ(ima_filt->ad, midsize, x + midsize - i, xsize),
		      dst, ysize - (taille - 1), (int) xsize, 1);

      if (x + i >= midsize && x + i < xsize + midsize)
	subinc_float (
		      MIJ(ima_filt->ad, midsize, x + i - midsize, xsize),
		      dst, ysize - (taille - 1), (int) xsize, 1);
    }
  }

  for (x = 0; x < midsize; x++) {
    setinc_float (MIJ(ima_filt->ad, 0, x, xsize), 0.F, ysize, (int) xsize);
    setinc_float (MIJ(ima_filt->ad, 0, xsize - x - 1, xsize), 0.F,
		  ysize, (int) xsize);
  }

  free ((void *) src);
  free ((void *) dst);
  return true;
}

/*
 * PROCEDURE	: firy_n_float
 *
 * INPUT       :
 * ima_in         Pointeur sur la pyramide de l'image.
 *
 * OUTPUT       :
 * ima_filt       Pointeur sur la pyramide de gradient spatial.
 *                On obtient les gradients suivant x ou y en fonction de
 *                la valeur des coefficients du filtre contenus dans "coefs".
 *
 * INPUTS      :
 * coefs          Pointeur sur les coefficients du filtre.
 * taille	 Taille du filtre.
 *
 * DESCRIPTION	:
 * La procedure effectue un filtrage taille X taille de
 * l'image "ima_in". Ce filtrage correspond au calcul d'un gradients,
 * suivant x ou y, selon la valeur des coefficients du filtre contenus
 * dans "coefs"
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
static	bool	firy_n_float (TImageShort *ima_in, TImageFloat *ima_filt,
			      float *coefs, size_t taille)
{
  size_t	midsize  = taille / 2;
  size_t	xsize	 = (size_t) ima_in->nbco;
  size_t	ysize	 = (size_t) ima_in->nbli;
  void		(*fir_float) (const float *, float *, size_t, const float *);
  float		*src;
  float		*dst;
  size_t	y;

  // Test si filtre applicable
  if ((xsize < midsize) || (ysize < midsize)) {
    return false;
  }

  switch (taille) {
  case 3	: fir_float = firf3sym_float; break;
  case 5	: fir_float = firf5sym_float; break;
  case 7	: fir_float = firf7sym_float; break;
  default : return false;
  }

  if ((src = (float *) malloc (xsize * sizeof(float))) == (float *) NULL)
    return false;

  if ((dst = (float *) malloc (xsize * sizeof(float))) == (float *) NULL) {
    free ((void *) src);
    return false;
  }

  set_float (ima_filt->ad, 0.F, xsize * ysize);

  for (y = 0; y < ysize; y++) {
    int	i;

    cast_short_float (MIJ(ima_in->ad, y, 0, xsize), src, xsize);

    for (i = 0; i < (int) midsize; i++) {
      (*fir_float) (src, dst, xsize - (taille - 1),
		    MIJ(coefs, i, 0, taille));

      if ((y + midsize >= (size_t) i) && (y + midsize < ysize + i))
	add_float (
		   MIJ(ima_filt->ad, y + midsize - i, midsize, xsize),
		   dst, xsize - (taille - 1));

      if (y + i >= midsize && y + i < ysize + midsize)
	subtract_float (
			MIJ(ima_filt->ad, y + i - midsize, midsize, xsize),
			dst, xsize - (taille - 1));
    }
  }

  for (y = 0; y < midsize; y++) {
    set_float (MIJ(ima_filt->ad, y, 0, xsize), 0.F, xsize);
    set_float (MIJ(ima_filt->ad, ysize - y - 1, 0, xsize), 0.F,
	       xsize);
  }

  free ((void *) src);
  free ((void *) dst);

  return true;
}
