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

  DESCRIPTION	: Le fichier contient la gestion de la compensation et du
                  recalage des images.

*/

/* Inclusion des fichiers standards.			*/
#include <memory.h>

/* Inclusion des fichiers generaux.			*/
#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"

/* Inclusion des prototypes des procedures internes.	*/
#include "compense.h"

/* Inclusion des prototypes des procedures locales.	*/
#include "../estimation/para_mvt.h"

#define DEBUG_LEVEL2 0
#define interp_possible(li,co,nbli,nbco) ((((int)li)>=0) && (((int)co)>=0) && \
					  (((int)li)<(nbli-1)) && \
                                          (((int)co)<(nbco-1)))

/*
 * PROCEDURE	  : set_ctes_warp_seq
 *
 * INPUTS        :
 * nbli             Nombre de lignes de la sequence recalee.
 * nbco             Nombre de colonnes de la sequence recalee.
 * offsetli         Offset en lignes dans l'image recalee.
 * offsetco         Offset en colonnes dans l'image recalee.
 * init_Ivariation  Indique si on doit initialiser la variation d'intensite.
 *                     - 0: pas d'initialisation.
 *                     - 1: remise a zero de la variable locale "Ivariation".
 *
 * OUTPUT         :
 * Nbli_ima_out     Nombre de lignes de l'image recalee.
 * Nbcol_ima_out    Nombre de colonnes de l'image recalee.
 * Offset_li        Decalage en lignes par rapport a l'origine.
 * Offset_co        Decalage en colonn par rapport a l'origine.
 * Ivariation       Variation d'intensite entre l'image de reference et
 *	            l'image traitee.
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 *
 * DESCRIPTION	  :
 * La procedure initialise les elements utiles a la compensation de la
 * sequence.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool set_ctes_warp_seq(int nbli, int nbco, int offsetli, int offsetco,
		       int init_Ivariation,
		       int *Nbli_ima_out, int *Nbcol_ima_out,
		       int *Offset_li, int *Offset_co, float *Ivariation,
		       TImageFloat *imFx, TImageFloat *imFy)
{
  float *ptfl;
  float  x,y;
  int    i,j;

  if(init_Ivariation != 0)
    *Ivariation = 0.0;

  *Offset_li = offsetli;
  *Offset_co = offsetco;
  if(( *Nbli_ima_out != nbli) || ( *Nbcol_ima_out!=nbco)) {
    /* demande  memoire pour conserver les deplacement cummules passes */
    *Nbli_ima_out = nbli;
    *Nbcol_ima_out= nbco;
    imFx->nbli = nbli;
    imFy->nbli = nbli;
    imFy->nbco = nbco;
    imFx->nbco = nbco;
    if(imFx->ad != NULL)
      free((char *)imFx->ad);
    if(imFy->ad != NULL)
      free((char *)imFy->ad);

    if((imFx->ad=(float *)malloc((unsigned)(nbli*nbco)*sizeof(float)))==NULL) {
      fprintf(stderr,"\nCan not allocate memory for imFx\n");
      return false;
    }

    /* Init avec l'application identite (Rq numero de colonne = x) */
    for(i = (*Offset_co); i < ( *Nbcol_ima_out + (*Offset_co)); i++) {
      x = (float)i;
      ptfl = imFx->ad - (*Offset_co) + i;
      for(j=0;j < *Nbli_ima_out;j++) {
	*ptfl = x;
	ptfl += nbco;   /* passage a la ligne suivante */
      }
    }

    if((imFy->ad=(float *)malloc((unsigned)(nbli*nbco)*sizeof(float)))==NULL) {
      fprintf(stderr,"\nCan not allocate memory for imFy\n");
      return false;
    }

    /* Init avec l'application identite (Rq numero de ligne = y) */
    for(i = (*Offset_li);i< ( *Nbli_ima_out + (*Offset_li)); i++) {
      y = (float)i;
      /* positionnement en debut de la i ieme ligne */
      ptfl = imFy->ad+(i- (*Offset_li)) *nbco;
      for(j = 0; j < *Nbcol_ima_out; j++)
	*ptfl++ = y;
    }
  }
  return true;
}



/*
 * PROCEDURE	: Recale_image
 *
 * INPUT       :
 * src		  Image a recaler.
 *
 * OUTPUT       :
 * dst		  Image de sortie recalee.
 *
 * INPUTS      :
 * ysize          Nombre de lignes de l'image d'entree a recaler.
 * xsize          Nombre de colonnes de l'image d'entree a recaler.
 *
 * OUTPUT         :
 * Nbli_ima_out     Nombre de lignes de l'image recalee.
 * Nbcol_ima_out    Nombre de colonnes de l'image recalee.
 * Ivariation       Variation d'intensite entre l'image de reference et
 *	            l'image traitee.
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 * DESCRIPTION	:
 * La procedure recale l'image d'entree "ima_in" a l'aide des deplacements
 * a appliquer a chaque pixel de l'image. Ces deplacements sont contenus dans
 * les variables locales "imFx" et "imFy".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
void Recale_image(unsigned char *src, unsigned char *dst, int ysize, int xsize,
		  int *Nbli_ima_out, int *Nbcol_ima_out, float *Ivariation,
		  TImageFloat *imFx, TImageFloat *imFy)
{
#define	BYTEMIN	0
#define	BYTEMAX	255

  double di = (double) (*Ivariation);
  int	 xend  = *Nbcol_ima_out;
  int	 yend  = *Nbli_ima_out;
  float	 *ptFx = imFx->ad;
  float	 *ptFy = imFy->ad;
  int    x, y;

  for (y = 0; y < yend; y++ )  {
    for (x = 0; x < xend; x++) {
      double fx =  (double) *ptFx;
      double fy =  (double) *ptFy;

      INTERPLT2D_DECLARE(xsize, fx, fy);

      if (INTERPLT2D_ISIN(xsize, ysize))  {
	double v;           /* valeur interpolee */
	double s;

	INTERPLT2D_PROCESS(unsigned char, (unsigned char *) src, v);
	s = v - di;
	if (s < BYTEMIN)	s = BYTEMIN;
	else if (s > BYTEMAX)	s = BYTEMAX;
	*dst = (unsigned char) s;
      }
      else  {
	/* x,y not in the image */
	*dst = 0;
      }
      dst++;
      ptFx++;
      ptFy++;
    }
  }
#undef	BYTEMIN
#undef	BYTEMAX
}

/*
 * PROCEDURE	: Recale_image
 *
 * INPUT       :
 * src		  Image a recaler.
 *
 * OUTPUT       :
 * dst		  Image de sortie recalee.
 *
 * INPUTS      :
 * ysize          Nombre de lignes de l'image d'entree a recaler.
 * xsize          Nombre de colonnes de l'image d'entree a recaler.
 *
 * OUTPUT         :
 * Nbli_ima_out     Nombre de lignes de l'image recalee.
 * Nbcol_ima_out    Nombre de colonnes de l'image recalee.
 * Ivariation       Variation d'intensite entre l'image de reference et
 *	            l'image traitee.
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 * DESCRIPTION	:
 * La procedure recale l'image d'entree "ima_in" a l'aide des deplacements
 * a appliquer a chaque pixel de l'image. Ces deplacements sont contenus dans
 * les variables locales "imFx" et "imFy".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
void Recale_image(short *src, short *dst, int ysize, int xsize,
		  int *Nbli_ima_out, int *Nbcol_ima_out, float *Ivariation,
		  TImageFloat *imFx, TImageFloat *imFy)
{
#define	BYTEMIN	0
#define	BYTEMAX	65535

  double di = (double) (*Ivariation);
  int	 xend  = *Nbcol_ima_out;
  int	 yend  = *Nbli_ima_out;
  float	 *ptFx = imFx->ad;
  float	 *ptFy = imFy->ad;
  int    x, y;

  for (y = 0; y < yend; y++ )  {
    for (x = 0; x < xend; x++) {
      double fx =  (double) *ptFx;
      double fy =  (double) *ptFy;

      INTERPLT2D_DECLARE(xsize, fx, fy);

      if (INTERPLT2D_ISIN(xsize, ysize))  {
	double v;           /* valeur interpolee */
	double s;

	INTERPLT2D_PROCESS(short, (short *) src, v);
	s = v - di;
	if (s < BYTEMIN)	s = BYTEMIN;
	else if (s > BYTEMAX)	s = BYTEMAX;
	*dst = (short) s;
      }
      else  {
	/* x,y not in the image */
	*dst = 0;
      }
      dst++;
      ptFx++;
      ptFy++;
    }
  }
#undef	BYTEMIN
#undef	BYTEMAX
}

/*
 * PROCEDURE	: copie_sup
 *
 * INPUT	:
 * src		Image des coefficients de ponderation.
 *
 * OUTPUT	:
 * dst		Image des coefficients mis a l'echelle.
 *
 * INPUT       :
 * win		Fenetre de travail.
 *
 * DESCRIPTION	:
 * La procedure met a l'echelle les coefficients de ponderation de l'image
 * "src" situes dans la fenetre "win". Les coefficients a l'echelle sont
 * stockes dans l'image "dst". La procedure dilate les cofficients de "src"
 * (compris entre 0 et 1) sur l'echelle des 256 niveaux de gris (compris entre
 * 0 et 255).
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void copie_sup (const TImageFloat *src, TImageShort *dst, const TWindow *win)
{
#define	BYTEMIN	0
#define	BYTEMAX	255

  size_t	xsize = src->nbco;	/* largeur des images		*/
  size_t	xinf  = win->dco;	/* limite horizontale inferieure*/
  size_t	xsup  = win->fco;	/* limite horizontale superieure*/
  size_t	yinf  = win->dli;	/* limite verticale inferieure	*/
  size_t	ysup  = win->fli;	/* limite verticale superieure	*/
  size_t	y;			/* indice de ligne		*/

  for (y = yinf; y < ysup; y++) {
    register const float	*sp   = MIJ(src->ad, y, xinf, xsize);
    register const float	*send = MIJ(src->ad, y, xsup, xsize);
    register short		*dp   = MIJ(dst->ad, y, xinf, xsize);

    for (; sp < send; sp++, dp++) {
      short	s = (short) ((float) BYTEMAX * *sp);

      if (s < BYTEMIN)	s = BYTEMIN;
      else if (s > BYTEMAX)	s = BYTEMAX;
      *dp = s;
    }
  }

#undef	BYTEMIN
#undef	BYTEMAX
}



/*
 * PROCEDURE	: Actualise_deplacements_para
 *
 * INPUT       :
 * param          Coefficients du modele de mouvement modelisant le champ
 *                des vitesses.
 *
 * OUTPUT         :
 * Nbli_ima_out     Nombre de lignes de l'image recalee.
 * Nbcol_ima_out    Nombre de colonnes de l'image recalee.
 * Offset_li        Decalage en lignes par rapport a l'origine.
 * Offset_co        Decalage en colonn par rapport a l'origine.
 * Ivariation       Variation d'intensite entre l'image de reference et
 *	            l'image traitee.
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 *
 * DESCRIPTION	:
 * La procedure actualise les deplacements, contenus dans les variables
 * locales "imFx" et "imFy", a appliquer a une image pour la recaler.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Actualise_deplacements_para (Para *param,
				  int *Nbli_ima_out, int *Nbcol_ima_out,
				  float *Ivariation,
				  TImageFloat *imFx, TImageFloat *imFy)
{

  double	coefs[MAXCOEFSMODEL];
  double	f[MAXCOEFSMODEL / 2];
  double	g[MAXCOEFSMODEL / 2];
  double	di;

  float		*ptFx = imFx->ad;
  float		*ptFy = imFy->ad;
  int		xend  = *Nbli_ima_out;
  int		yend  = *Nbcol_ima_out;
  int		x, y;


  /* obtient les parametres dans le repere (0, 0)	*/
  if (chgt_repere(param, coefs, 0, 0) == false)
    return false;

  /* recupere les coefficients du modele	*/
  mvtcoef_to_poly (coefs, param->var_light, f, g, &di);

  /* parametres de deplacement, pas de mouvement	*/
  f[1]++, g[2]++;

  /* ajuste la variation d'intensite	*/
  *Ivariation += (float) di;

  if (model_degree(param->id_model)==2) {
    for (y = 0; y < yend; y++)
      for (x = 0; x < xend; x++) {
	double	x_ = (double) *ptFx;
	double	y_ = (double) *ptFy;

	double dx = f[0]
	  + x_ * (f[1] + (x_ * f[3]) + (y_ * f[4]))
	  + y_ * (f[2] + (y_ * f[5]));
	double dy = g[0]
	  + x_ * (g[1] + (x_ * g[3]) + (y_ * g[4]))
	  + y_ * (g[2] + (y_ * g[5]));
	*ptFx++ = (float) dx;
	*ptFy++ = (float) dy;
      }
  }
  else {
    for (y = 0; y < yend; y++)

      for (x = 0; x < xend; x++) {
	double	x_ = (double) *ptFx;
	double	y_ = (double) *ptFy;

	double dx = f[0] + (x_ * f[1]) + (y_ * f[2]);
	double dy = g[0] + (x_ * g[1]) + (y_ * g[2]);
	*ptFx++ = (float) dx;
	*ptFy++ = (float) dy;
      }
  }
  return true;
}



/*
 * PROCEDURE	: Back_Warp_ima
 *
 * INPUTS      :
 * src            Image a recaler.
 * ysize          Nombre de lignes de l'image d'entree a recaler.
 * xsize          Nombre de colonnes de l'image d'entree a recaler.
 *
 * OUTPUT       :
 * dst            Image de sortie recalee.
 *
 * INPUT       :
 * param          Coefficients du modele de mouvement. Ces coefficients
 *                representent le mouvement cumule depuis la premiere
 *                image de la sequence.
 *
 * DESCRIPTION	:
 * La procedure recale l'image d'entree "ima_in" a l'aide du modele de
 * mouvement "param", estime entre l'image "ima_in" et la premiere
 * image de la sequence. En fait "param" contient les coefficients d'un
 * modele prenant en compte les mouvements cumules depuis la premiere
 * image de la sequence.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
bool Back_Warp_ima(const short *src, int ysize, int xsize, short *dst,
		   Para *param)
{
#define	BYTEMIN	0
#define	BYTEMAX	255

  short		*ptr_int, *im_int;	/* Image intermediaire. */
  double	coefs[MAXCOEFSMODEL];
  double	f[MAXCOEFSMODEL / 2];
  double	g[MAXCOEFSMODEL / 2];
  double	di;

  int		x, y;

  /* Obtient les parametres dans le repere (0, 0).	*/
  if (chgt_repere(param, coefs, 0, 0) == false)
    return false;

  mvtcoef_to_poly (coefs, param->var_light, f, g, &di);

  /* Parametres de deplacement, pas de mouvement.	*/
  f[1]++, g[2]++;

  /* Utilisation d'une image intermediaire (si src = dst) */
  im_int = (short *) calloc((size_t) (xsize * ysize), sizeof(short));
  ptr_int = im_int;

  /*
   * On simplifie les calculs si le nombre de parametres est inferieur ou
   * egal a 7.
   */
  if (model_degree(param->id_model)==2) {
    for (y = 0; y < ysize; y++)
    {
      double y_ = (double) y;

      for (x = 0; x < xsize; x++)
      {
	double x_ = (double) x;

	double dx = f[0]
	  + x_ * (f[1] + (x_ * f[3]) + (y_ * f[4]))
	  + y_ * (f[2] + (y_ * f[5]));
	double dy = g[0]
	  + x_ * (g[1] + (x_ * g[3]) + (y_ * g[4]))
	  + y_ * (g[2] + (y_ * g[5]));

	INTERPLT2D_DECLARE(xsize, dx, dy);

	if (INTERPLT2D_ISIN(xsize, ysize))
	{
	  double v;           /* valeur interpolee */
	  short  s;

	  INTERPLT2D_PROCESS(short, (short *) src, v);
	  s = (short) (v - di);
	  if (s < BYTEMIN)	s = BYTEMIN;
	  else if (s > BYTEMAX)	s = BYTEMAX;
	  *ptr_int = s;
	}
	else
	{
	  /* x,y not in the image */
	  *ptr_int = 0;
	}
	ptr_int++;
      }
    }
  }
  else
  {
    for (y = 0; y < ysize; y++)
    {
      double y_ = (double) y;
      for (x = 0; x < xsize; x++)
      {
	double x_ = (double) x;

	double dx = f[0] + (x_ * f[1]) + (y_ * f[2]);
	double dy = g[0] + (x_ * g[1]) + (y_ * g[2]);

	INTERPLT2D_DECLARE(xsize, dx, dy);

	if (INTERPLT2D_ISIN(xsize, ysize))
	{
	  double v;           /* valeur interpolee */
	  short  s;

	  INTERPLT2D_PROCESS(short, (short *) src, v);
	  s = (short) (v - di);
	  if (s < BYTEMIN)	s = BYTEMIN;
	  else if (s > BYTEMAX)	s = BYTEMAX;
	  *ptr_int = s;
	}
	else
	{
	  /* x,y not in the image */
	  *ptr_int = 0;
	}
	ptr_int++;
      }
    }
  }

  /* Copie de l'image intermediaire vers la destination. */
  (void) memcpy ((void *) dst, (void *) im_int,
		 xsize * ysize * sizeof(short));

  free((char *)im_int);

  return true;
#undef	BYTEMIN
#undef	BYTEMAX
}


/*
 * PROCEDURE	: Compose_para
 *
 * INPUTS      :
 * par1           Premiers parametres.
 * par2           Deuxiemes parametres.
 *
 * OUTPUT       :
 * par_out        Parametre compose: par_out = par2 o par1
 *
 * DESCRIPTION	:
 * La procedure compose deux mouvements "par1" et "par2" pour obtenir
 * le mouvement resultant "par_out".
 * Pour l'instant, on est limite a un nbre de parametres inferieur a 7.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool compose_para(Para *par1, Para *par2, Para *par_out)
{
  double        A1[2][2],A2[2][2],A_out[2][2],V1[2],V2[2],V_out[2];
  double        thet1[13], thet2[13], thet_out[13];
  int		i,j,k;

  for(i=0; i < MAXCOEFSMODEL; i++) {
    par_out->thet[i] = thet_out[i] = 0.0;
  }

  for(i=0;i<2;i++) {
    for(j=0;j<2;j++) {
      A1[i][j] = 0.0;
      A2[i][j] = 0.0;
      A_out[i][j] = 0.0;
    }
    V1[i] = 0.0;
    V2[j] = 0.0;
    V_out[i] = 0.0;
  }

  /* on transforme tous les parametres dans le repere 0 */
  if (chgt_repere(par1, thet1, 0, 0) == false) return false;
  if (chgt_repere(par2, thet2, 0, 0) == false) return false;

  int degre_par1 = model_degree(par1->id_model);
  int degre_par2 = model_degree(par2->id_model);

  // on veut le modele, pas la facon dont sont stockes les coefs
  switch (degre_par1)
  {
  case 2: // Quadratic model
    printf("\n\n\nError : quadratic model no composition possible...");
    return false;
    //    break;
  case 1: // Affine model
    A1[0][0] = thet1[2];
    A1[0][1] = thet1[3];
    A1[1][0] = thet1[4];
    A1[1][1] = thet1[5];
  case 0: // Constant model
    V1[0] = thet1[0];
    V1[1] = thet1[1];
    A1[0][0] += 1.0;
    A1[1][1] += 1.0; // on calcule les parametres de deplacement
    break;
  }

  switch (degre_par2)
  {
  case 2: // Quadratic model
    printf("\n\n\nError : quadratic model no composition possible...");
    return false;
    //    break;
  case 1: // Affine model
    A2[0][0] = thet2[2];
    A2[0][1] = thet2[3];
    A2[1][0] = thet2[4];
    A2[1][1] = thet2[5];
  case 0: // Constant model
    V2[0] = thet2[0];
    V2[1] = thet2[1];
    A2[0][0] += 1.0;
    A2[1][1] += 1.0; // on calcule les parametres de deplacement
    break;
  }

  /* calcul de la composee dans le repere 0,0 */
  /* A_out = A2*A1,   V_out = A2*V1 + V2 */
  for(i=0;i<2;i++)
    {
      for(j=0;j<2;j++)
	{
	  for(k=0;k<2;k++)
	    A_out[i][j] += A2[i][k]*A1[k][j];
	  V_out[i] += A2[i][j]*V1[j];
	}
      V_out[i] += V2[i];
    }

  /* Ecriture des parametres dans par_out */
  par_out->thet[0] = V_out[0];
  par_out->thet[1] = V_out[1];
  par_out->thet[2] = A_out[0][0];
  par_out->thet[3] = A_out[0][1];
  par_out->thet[4] = A_out[1][0];
  par_out->thet[5] = A_out[1][1];
  par_out->thet[2] -= 1.0;
  par_out->thet[5] -= 1.0; /* on recherche les para de vitesse  cas          */
                           /* particulier, si 4 paras, inverser un coef, les */
                           /* suivants peuvent etre mis a zero               */

  par_out->nb_para = Min(6,Max(par1->nb_para, par2->nb_para));


  // pour l'instant on deduit le modele du degre des 2 modeles a composer.
/*    if ((degre_par1 == 1) || (degre_par2 == 1)) */
/*      par_out->id_model = MDL_AFF_COMPLET; */
/*    else  */
/*      par_out->id_model = MDL_TR; */
  if (degre_par1 != degre_par2) {
    if ( (degre_par1 == 1) || (degre_par2 == 1) )
      par_out->id_model = MDL_AFF_COMPLET;
    else
      par_out->id_model = MDL_TR;
  }
  else
    par_out->id_model = par1->id_model;

  if (par1->var_light || par2->var_light )
  {
    par_out->var_light = true;
    par_out->thet[12] = par1->thet[12] + par2->thet[12];
  }
  else
    par_out->var_light = false;

  double li_c = par_out->li_c;
  double co_c = par_out->co_c;
  par_out->li_c = 0;
  par_out->co_c = 0;


  if (chgt_repere(par_out, thet_out, li_c, co_c) == false)
    return false;

  for(i=0; i < MAXCOEFSMODEL; i++) {
    par_out->thet[i] = thet_out[i];
  }

  par_out->li_c = li_c;
  par_out->co_c = co_c;

  return true;
}



/*
 * PROCEDURE	: masque_suivant
 *
 * INPUTS      :
 * segm_in        Image contenant le masque a projeter.
 * num_reg        Le masque contenu dans "masque_in" porte l'etiquette
 *                "num_reg".
 *
 * OUTPUT       :
 * segm_out       Image contenant le masque projete dans le sens du mouvement.
 *
 * INPUTS      :
 * num_reg_out    Valeur de l'etiquette du masque "masque_out" projete dans
 *                le sens du mouvement.
 * nbli           Nombre de lignes de l'image "masque_in".
 * nbco           Nombre de colonnes de l'image "masque_in".
 * param          Parametres estimes du modele de mouvement.
 *
 * DESCRIPTION	:
 * La procedure determine le masque suivant (points a prendre en compte lors de
 * l'estimation du mouvement), en fonction des parametres du modele.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool masque_suivant(short *segm_in, int num_reg, short *segm_out,
		    int num_reg_out, int nbli, int nbco, Para *param)
{
  double
    thet[13],
    a=0., b=0., c=0.,  d=0.,      /* linear coefficients */
    u=0.,v=0.,                    /* translation vector */
    q1 =0.,q2=0.0,q3=0.,          /* coefficients quadratique */
    q4=0.,q5=0.,q6=0.,
    x, y,
    i_dbl,  j_dbl;

  int
    i, j,x_,y_;

  short *ptr_int;
  short *ptr,*ptr_segm_in,*masq_int;
  int   npixels;

  /* Obtention des parametres dans le repere (0,0) */
  if (chgt_repere(param,thet,0,0) == false)
    return false;

  u = thet[0];
  v = thet[1];

  a = thet[2];
  b = thet[3];
  c = thet[4];
  d = thet[5];

  q1 = thet[6];
  q2 = thet[7];
  q3 = thet[8];
  q4 = thet[9];
  q5 = thet[10];
  q6 = thet[11];

  /* on veut les parametres de deplacement, pas de mouvement */
  a += 1.0;  d += 1.;

  if(num_reg<0){
    fprintf(stderr, "Bad etiq for the next support computation\n");
    return false;
  }

  /* Utilisation d'un masque intermediaire (si masq_in = masq_out) */
  masq_int = (short *)malloc((unsigned)(nbli*nbco)*sizeof(short));
  npixels = nbli*nbco;
  ptr_int = masq_int+npixels-1;
  while(ptr_int >=  masq_int)
    *ptr_int --=-100;

  ptr_segm_in = segm_in;
  for (j = 0; j < nbli;j++ )
    {
      j_dbl = (double) j;
      for (i = 0; i < nbco;i++,ptr_segm_in++)
	{
	  if(*ptr_segm_in == num_reg)
	    {
	      i_dbl = (double) i;

	      x =  u + i_dbl*(a+q1*i_dbl+q2*j_dbl)+j_dbl*(b+q3*j_dbl);
	      y =  v + i_dbl*(c+q4*i_dbl+q5*j_dbl)+j_dbl*(d+q6*j_dbl);

	      /* On place tout les points autour de x,y qui */
	      /* sont dans l'image, a num_reg.              */
	      x_ = (int)x;
	      y_ = (int)y;
	      if(INTERPLT2D_POSSIBLE(y_,x_,nbli,nbco))
		{
		  ptr_int = masq_int+y_*nbco+x_;
		  *ptr_int++ = (short) num_reg;
		  *ptr_int   = (short) num_reg;
		  ptr_int   += nbco;
		  *ptr_int-- = (short) num_reg;
		  *ptr_int   = (short) num_reg;
		}
	    }
	}
    }
  /* Recopie de la region projetee dans l'image de sortie */
  ptr_int = masq_int;
  ptr = segm_out;
  for(i=0;i<npixels;i++,ptr++,ptr_int++)
    {
      if(*ptr_int == num_reg)
	*ptr = (short) num_reg_out;
      else
	*ptr = ETIQELIM;
    }

  free((char *)masq_int);

  return true;
}



/*
 * PROCEDURE	: free_recalage_film
 *
 * INPUTS      :
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 *
 * DESCRIPTION	:
 * La procedure libere la memoire allouee pour la compensation du mvt.
 *
 * HISTORIQUE   :
 * 1.00 - 05/03/98 - Original.
 */
void free_recalage_film(TImageFloat *imFx, TImageFloat *imFy)
{
  if (DEBUG_LEVEL2) printf("----------------- libere imFx->ad\n");
  free((float *) imFx->ad);
  free((float *) imFy->ad);
  imFx->ad = (float *) NULL;
  imFy->ad = (float *) NULL;
}


/*
 * PROCEDURE	: get_deplacement
 *
 * INPUTS      :
 * Nbcol_ima_out    Nombre de colonnes de l'image recalee.
 * imFx, imFy	    Deplacements a appliquer a chaque pixel pour recaler
 *                  une image.
 *
 * DESCRIPTION	:
 * La procedure determine le deplacement a appliquer au pixel (x, y)
 * pour compenser son mouvement.
 *
 * HISTORIQUE   :
 * 1.00 - 05/03/98 - Original.
 */
void get_deplacement (int x, int y, float *xdepl, float *ydepl,
		      int *Nbcol_ima_out, TImageFloat *imFx, TImageFloat *imFy)
{
  size_t offset = y * (*Nbcol_ima_out) + x;

  *xdepl = (* (imFx->ad + offset));
  *ydepl = (* (imFy->ad + offset));
}


/*
 * PROCEDURE	: Warp_region
 *
 * INPUTS      :
 * segm_in        Masque a projeter dans le sens du mouvment.
 * num_reg	  Numero de la region a projeter dans le sens du mouvment.
 * num_reg_out	  Numero de la region projetee dans le sens du mouvment.
 *		  La region est projetee avec ce numero d'etiquette.
 * nbli           Nombre de lignes du masque a projeter.
 * nbco           Nombre de colonnes du masque a projeter.
 *
 * OUTPUT       :
 * segm_out       Masque projete dans le sens du mouvment.
 *
 * INPUT       :
 * param          Coefficients du modele de mouvement. Ces coefficients
 *                representent le mouvement cumule depuis la premiere
 *                image de la sequence.
 * v_init_def	  Valeur des points qui n'ont pas d'antecedents.
 *
 * DESCRIPTION	:
 * La procedure determine le masque suivant en fonction des parametres de
 * mouvement.
 *
 * HISTORIQUE   :
 * 1.00 - 21/10/97 - Original.
 */
bool Warp_region(short *segm_in, int num_reg, int num_reg_out,
		 short *segm_out, int nbli, int nbco, Para *param)
{
  double
    thet[13],
    a=0., b=0., c=0.,  d=0.,      /* linear coefficients */
    u=0.,v=0.,                    /* translation vector */
    q1 =0.,q2=0.0,q3=0.,          /* coefficients quadratique */
    q4=0.,q5=0.,q6=0.,
    x, y,
    i_dbl,  j_dbl;

  int
    i, j,x_,y_;

  short *ptr_int;
  short *ptr,*ptr_segm_in,*masq_int;

  int npixels;

  /* Obtention des parametres dans le repere (0,0) */
  if (chgt_repere(param,thet,0,0) == false)
    return false;

  u = thet[0];
  v = thet[1];

  a = thet[2];
  b = thet[3];
  c = thet[4];
  d = thet[5];

  q1 = thet[6];
  q2 = thet[7];
  q3 = thet[8];
  q4 = thet[9];
  q5 = thet[10];
  q6 = thet[11];

  /* on veut les parametres de deplacement, pas de mouvement */
  a += 1.0;  d += 1.;

  if(num_reg<0){
    fprintf(stderr, "Bad etiq for the warping\n");
    return false;
  }

  /* Utilisation d'un masque intermediaire (si masq_in = masq_out) */
  masq_int = (short *)malloc((unsigned)(nbli*nbco)*sizeof(short));
  npixels = nbli*nbco;
  ptr_int=masq_int+npixels-1;
  while(ptr_int>=masq_int)
    *ptr_int--=-100;

  ptr_segm_in = segm_in;
  for (j = 0; j < nbli;j++ ){
    j_dbl = (double) j;
    for (i = 0; i < nbco;i++,ptr_segm_in++){
      if(*ptr_segm_in == num_reg) {
	i_dbl = (double) i;

	x =  u + i_dbl*(a+q1*i_dbl+q2*j_dbl)+j_dbl*(b+q3*j_dbl);
	y =  v + i_dbl*(c+q4*i_dbl+q5*j_dbl)+j_dbl*(d+q6*j_dbl);

	/* On place tout les points autour de x,y qui sont dans l'image
	 * a num_reg. */
	x_ = (int)x; y_ = (int)y;
	if(INTERPLT2D_POSSIBLE(y_,x_,nbli,nbco)){
	  ptr_int = masq_int+y_*nbco+x_;
	  *ptr_int++ = (short) num_reg; *ptr_int = (short) num_reg; ptr_int += nbco;
	  *ptr_int-- = (short) num_reg; *ptr_int = (short) num_reg;
	}
      }
    }
  }
  /* Recopie de la region projetee dans l'image de sortie */
  ptr_int = masq_int; ptr = segm_out;
  for(i=0;i<npixels;i++,ptr++,ptr_int++)
    if (*ptr_int == num_reg) *ptr = (short) num_reg_out;

  free((char *)masq_int);

  return true;
}

