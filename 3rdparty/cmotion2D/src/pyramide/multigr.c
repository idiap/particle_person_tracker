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
                  des pyramides d'images avec filtrage et sous-echantionnage.

*/


/* Inclusion des fichiers generaux                  */
#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"

/* Inclusion des prototypes des procedures internes */
#include "multigr.h"

/* Inclusion des prototypes des procedures locales  */
#include "memoire.h"
#include "acast.h"
#include "famem.h"
#include "faarith.h"
#include "fafirf3.h"
#include "fafirf5.h"
#include "fafirf7.h"

/*
 * PROCEDURE	: Reduit_pyramide
 *
 * INPUTS      :
 * imag           Pointeur sur l'image de base de la pyramide.
 * niveau_max     Niveau maximal de la pyramide a construire
 * coef           Coefficient du filtre utilise avant le sous-echantillonnage.
 * mem            Indique si la memoire utile pour construire la pyramide
 *                a ete allouee. Si mem = 1, la memoire est a reserver.
 *
 * DESCRIPTION	:
 * La procedure construit la pyramide de l'image de base de short.
 * Elle calcule toutes les images reduites jusqu'a niveau_max.
 * Cette procedure suppose que la memoire a deja ete allouee si mem vaut 0,
 * et quelle doit etre alloue si mem = 1.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Reduit_pyramide(TImageShort *imag, int & niveau_max, double coef, int mem)
{
  int	i;
  bool state;
  int niv_max = niveau_max;
  if(mem) {
    // Warning: modification eventuelle de niv_max
    state = Mem_pyramide(&imag[1],imag[0].nbli/2,imag[0].nbco/2, niv_max);
    if (state == false) {
      // Aucun niveau n'a pu etre alloue.
      return false;
    }
  }
  for(i=1;i<=niv_max;i++) {
    state = reduit_si(&imag[i-1],&imag[i],coef);
    if (state == false) {
      // Erreur lors de la construction du niveau i
      niv_max = i - 1;
      break;
    }      
  }

  niveau_max = niv_max;
  
  return true;
}



/*
 * PROCEDURE	: reduit_si
 *
 * INPUT       :
 * ima_in	  Pointeur sur une image de la pyramide.
 *
 * OUTPUT       :
 * ima_out	  Pointeur sur l'image reduite, filtree et sous-echantionnee
 *                a un niveau superieur de la pyramide.
 *
 * INPUT       :
 * coef           Coefficient du filtre utilise avant le sous-echantillonnage.
 *                Si coef = 0, on prend 0.4 comme coefficient du filtre de
 *                lissage. Sinon on prend la valeur de coef.
 *
 * DESCRIPTION	:
 * La procedure calcule l'image reduite, filtree et sous-echantionnee
 * a un niveau superieur de la pyramide.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool reduit_si(const TImageShort *ima_in, TImageShort *ima_out, double coef)
{
#define	FMAXSIZE	5

  size_t	xsize  = ima_in->nbco;
  size_t	ysize  = ima_in->nbli;
  size_t	mxsize = ima_out->nbco;
  size_t	mysize = ima_out->nbli;

  void	(*fir_float) (const float *, float *, size_t, const float *);

  float	fir[FMAXSIZE];
  float	pond[FMAXSIZE / 2];
  float	w;
  size_t	fsize;	/* taille du filtre			*/
  size_t	msize;	/* moitiee de la taille du filtre	*/
  float	*ima;	/* image temporaire			*/
  float	*src;	/* tableau source			*/
  float	*dst;	/* tableau destination			*/
  size_t	size;	/* taille intermediaire			*/
  size_t	x, y;	/* indice colonne et ligne		*/
  size_t	i;

	/* coefficients du filtre	*/
  fsize = FMAXSIZE;
  msize = fsize / 2;

  // Test si filtre taille 5 applicable
  if ((xsize < msize) || (ysize < msize)) {
    // On reduit la taille du filtre a 3
    fsize = 3;
    msize = fsize / 2;
    // Si la taille du filtre est encore trop importante, on arrete
    if ((xsize < msize) || (ysize < msize)) {
      return false;
    }
  }


  fir[1] = fir[3] = 0.25F;
  if (coef != 0.0) {
    fir[2] = (float) coef;
    fir[0] = fir[4] = 0.25F - (float) (coef / 2.0);
  }
  else {
    fir[2] = 0.4F;
    fir[0] = fir[4] = 0.05F;
  }

  /* convolution par filtre separable	*/
  w = 0.5F + (fir[msize] / 2.F);
  for (i = 0; i < msize; i++) {
    pond[i] = w;
    w += fir[msize + i + 1];
  }

  switch (fsize) {
  case 3	: fir_float = firf3sym_float; break;
  case 5	: fir_float = firf5sym_float; break;
  case 7	: fir_float = firf7sym_float; break;
  default : return false;
  }

  size = xsize * ysize;
  if ((ima = (float *) malloc (size * sizeof(float))) == (float *) NULL)
    return false;

  size = Max(xsize, ysize) + fsize;
  src = (float *) malloc (size * sizeof(float));
  dst = (float *) malloc (size * sizeof(float));

  /* filtrage en ligne	*/
  set_float (src, 0.F, size);
  for (y = 0; y < ysize; y++) {
    float	*tmp = dst;

    cast_short_float (MIJ(ima_in->ad, y, 0, xsize), &src[msize],
		      xsize);

    dst = MIJ(ima, y, 0, xsize);
    (*fir_float) (src, dst, xsize, fir);

    for (i = 0; i < msize; i++) {
      dst[i] /= pond[i];
      dst[xsize - i - 1] /= pond[i];
    }
    dst = tmp;
  }

  /* filtrage en colonne	*/
  set_float (src, 0.F, size);
  for (x = 0; x < mxsize; x++) {
    short	*out = MIJ(ima_out->ad, 0, x, mxsize);

    copyinc_float (MIJ(ima, 0, x * 2, xsize), &src[msize],
		   ysize, (int) xsize, 1);
    (*fir_float) (src, dst, ysize, fir);

    for (i = 0; i < msize; i++) {
      dst[i] /= pond[i];
      dst[ysize - i - 1] /= pond[i];
    }
    for (i = 0; i < mysize; i++) {
      *out = (short) (dst[i * 2] + 0.5F);
      out += mxsize;
    }
  }

  free ((void *) src);
  free ((void *) dst);
  free ((void *) ima);

  return true;
}


/*
 * PROCEDURE	: ss_echant_pyramide
 *
 * INPUT       :
 * imag           Pointeur sur l'image a sous-echantillonner.
 *
 * OUTPUT       :
 *
 * DESCRIPTION	:
 * La procedure echantillonne imag d'un facteur 1/2 en ligne et en colonne.
 */
void ss_echant_pyramide(TImageShort *imag, int niv_max)
{
  int	i;
  for(i = 1;i <= niv_max; i++)
    ss_echantillonne(&imag[i-1], &imag[i]);
}

/*
 * PROCEDURE	: ss_echantillonne
 *
 * INPUT       :
 * ima_in         Pointeur sur l'image a sous-echantillonner.
 *
 * OUTPUT       :
 * ima_out        Pointeur sur l'image sous-echantillonnee.
 *
 * INPUT       :
 * creation_ima_out Indique s'il faut creer "ima_out".
 *                  Si "creation_ima_out" = 1, l'image "ima_out" est creee,
 *                  (avec allocation memoire).
 *
 *
 * DESCRIPTION	:
 * La procedure echantillonne ima_in d'un facteur 1/2 en ligne et en colonne.
 * Si ima_out n'a pas ete creee, la procedure le fait si l'on place
 * creation_ima_out a 1.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void ss_echantillonne(TImageShort *ima_in, TImageShort *ima_out)
{
  int     i,j;
  short   *p_in,*p_out;

  for(j=0;j<ima_out->nbli;j++)
    {
      p_in = ptrimage_l(ima_in,2*j);
      p_out =ptrimage_l(ima_out,j);
      for(i=0;i<ima_out->nbco;i++,p_out++)
	{
	  *p_out = *p_in;
	  p_in += 2;
	}
    }
}













