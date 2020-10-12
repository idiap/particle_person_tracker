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

  DESCRIPTION	: Le fichier contient la gestion du filtrage gaussien
                  des images de base.

*/


/* Inclusion des fichiers generaux.			*/
#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"

/* Inclusion des prototypes des procedures internes.	*/
#include "filt_gauss.h"

#include "acast.h"
#include "famem.h"
#include "faarith.h"
#include "fafirf3.h"
#include "fafirf5.h"
#include "fafirf7.h"

/* Declaration des variables locales.			*/
static int INIT_FILTRE = FALSE;   /* indique si le filtre a ete initialise */
static FILTRE_GAUSS filt_gauss;   /* coeffients du filtre gaussien         */


/*
 * PROCEDURE	: filtre_gauss
 *
 * INPUT       :
 * ima_in         Pointeur sur l'image a filtrer.
 *
 * OUTPUT       :
 * ima_out        Pointeur sur l'image filtree.
 *
 * INPUT       :
 * sigma2         Pointeur sur la valeur de la variance du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre l'image "ima_in" par un filtre gaussien de variance
 * "sigma2".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool filtre_gauss (const TImageShort *ima_in, TImageShort *ima_out,
		   const double *sigma2)
{
#define	XFMAXSIZE	7
#define	YFMAXSIZE	7

  size_t	xsize = ima_in->nbco;
  size_t	ysize = ima_in->nbli;

  void	(*fir_float) (const float *, float *, size_t, const float *);

  float	fir[YFMAXSIZE * XFMAXSIZE];
  size_t	fsize;	/* taille du filtre			*/
  size_t	msize;	/* moitiee de la taille du filtre	*/
  float	*ima;	/* image temporaire			*/
  float	*src;	/* tableau source			*/
  float	*dst;	/* tableau destination			*/
  size_t	size;	/* taille intermediaire			*/
  size_t	x, y;	/* indice colonne et ligne		*/

	/* on ne calcule qu'une seule fois les coefficients du filtre	*/
  if (calc_coef_filtre_gauss (&filt_gauss, sigma2) == false)
    return false;

  /* Convolution par filtre separable   */
  fsize = filt_gauss.taille;
  msize = filt_gauss.nindex;

  switch (fsize) {
  case 3	: fir_float = firf3sym_float; break;
  case 5	: fir_float = firf5sym_float; break;
  case 7	: fir_float = firf7sym_float; break;
  default : return false;
  }

  cast_double_float (filt_gauss.ad_coef, fir, fsize);

  size = xsize * ysize;
  if ((ima = (float *) malloc (size * sizeof(float))) == (float *) NULL) {
    fprintf(stderr, "Can not allocate memory for the Gaussian filter.\n");
    return false;
  }

  size = Max(xsize, ysize) + fsize;
  src = (float *) malloc (size * sizeof(float));
  dst = (float *) malloc (size * sizeof(float));

  /* filtrage en ligne	*/
  set_float (src, 0.f, size);
  for (y = 0; y < ysize; y++) {
    float	*tmp = dst;
    size_t	i;

    cast_short_float (MIJ(ima_in->ad, y, 0, xsize), &src[msize],
		      xsize);

    dst = MIJ(ima, y, 0, xsize);
    (*fir_float) (src, dst, xsize, fir);

    for (i = 0; i < msize; i++) {
      float	norm = (float) filt_gauss.ad_pond[i];

      dst[i] /= norm;
      dst[xsize - i - 1] /= norm;
    }
    dst = tmp;
  }

  /* filtrage en colonne	*/
  set_float (src, 0.f, size);
  for (x = 0; x < xsize; x++) {
    short	*out = MIJ(ima_out->ad, 0, x, xsize);
    size_t	i;

    copyinc_float (MIJ(ima, 0, x, xsize), &src[msize],
		   ysize, (int) xsize, 1);
    (*fir_float) (src, dst, ysize, fir);

    for (i = 0; i < msize; i++) {
      float	norm = (float) filt_gauss.ad_pond[i];

      dst[i] /= norm;
      dst[ysize - i - 1] /= norm;
    }
    for (i = 0; i < ysize; i++) {
      *out = (short) (dst[i] + 0.5F);
      out += xsize;
    }
  }

  free ((void *) src);
  free ((void *) dst);
  free ((void *) ima);

  return true;
}



/*
 * PROCEDURE	: calc_coef_filtre_gauss
 *
 * INPUT       :
 * filtre         Pointeur sur le filtre a creer.
 *
 * INPUTS	:
 * sigma2         Valeur de la variance du filtre gaussien.
 *
 * DESCRIPTION	:
 * La procedure calcule les coefficients d'une filtre gaussien de variance
 * "sigma2". La procedure renvoie 1 si de nouveaux coefficients ont ete
 * calcules. Sinon elle renvoie 0.
 *
 * HISTORIQUE	:
 * 1.00 - 01/01/95 - Original.
 */

bool calc_coef_filtre_gauss(FILTRE_GAUSS *filtre, const double *sigma2)
{
  double gauss[101], v_filt, gauss_pd = 1.0;
  int	 i, nindex;

  filtre->flag = INIT_FILTRE;
  if ((filtre->flag) && (filtre->variance != *sigma2))
  { /* un filtre existe deja, mais sa variance n'est pas la meme */
    filtre->variance = *sigma2;
    filtre->flag = FALSE;
  }
  if(filtre->flag == FALSE)    /* il faut generer un nouveau filtre */
  {
    /* Calcul des coefficients du filtre */
    nindex = 0;
    v_filt = 1.0;
    while(v_filt>0.05)
    {
      if(nindex==0)
      {
	gauss[50] = 1.0;
	gauss_pd = gauss[50];
      }
      else
      {
	gauss[50-nindex] = v_filt;
	gauss[50+nindex] = v_filt;
	/* gauss_pd contient la somme des coefs du filtre */
	gauss_pd = gauss_pd + 2.*v_filt;
      }
      nindex++;
      v_filt = exp(-((double)(nindex)*(double)(nindex))/(2.* *sigma2));
    }
    nindex--;

      /* initialisation de la structure FILTRE_GAUSS */
    filtre->variance = *sigma2;
    filtre->taille = 2*nindex + 1;
    filtre->nindex = nindex;

    /* allocation memoire pour les coefficients du filtre gaussien */
    if( (filtre->ad_coef = (double *)calloc((size_t) filtre->taille,
					    sizeof(double))) == NULL)
    {
      fprintf(stderr, "Can not allocate memory for the Gaussian filter.\n");
      return (false);
    }
    *(filtre->ad_coef+nindex) = gauss[50] / gauss_pd;
    for (i=1; i<=nindex; i++)
    {
      *(filtre->ad_coef+nindex-i) = gauss[50-i] / gauss_pd;
      *(filtre->ad_coef+nindex+i) = gauss[50+i] / gauss_pd;
    }

    /* allocation memoire pour la ponderation du filtre gaussien */
    if( (filtre->ad_pond = (double *)calloc((size_t) filtre->nindex,
					    sizeof(double))) == NULL)
    {
      fprintf(stderr, "Can not allocate memory for the Gaussian filter.\n");
      return (false);
    }
    gauss_pd = .5 + (*(filtre->ad_coef+nindex))/2.0;
    for (i=0; i<nindex; i++)
    {
      *(filtre->ad_pond+i) = gauss_pd;
      gauss_pd += *(filtre->ad_coef+nindex+i+1);
    }
    /* fin de calcul des coefficients du filtre */

    filtre->flag = TRUE;
    INIT_FILTRE = TRUE;
  }

  return true;
}


/*
 * PROCEDURE	: efface_memoire_filtre_gauss
 *
 * INPUT       : AUCUNE.
 *
 * OUTPUT       : AUCUNE.
 *
 * DESCRIPTION	:
 * La procedure efface la memoire allouee au coefficients du filtre gaussien.
 *
 * HISTORIQUE	:
 * 1.00 - 01/01/95 - Original.
 */

void efface_memoire_filtre_gauss()
{
  free(filt_gauss.ad_coef);
  free(filt_gauss.ad_pond);
  filt_gauss.ad_coef = NULL;
  filt_gauss.ad_pond = NULL;
  INIT_FILTRE = FALSE;
}




