/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef compense_h
#define compense_h

#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif

#include "../inc/interplt.h"
#include "../estimation/para_mvt.h"

bool set_ctes_warp_seq(int nbli, int nbco, int offsetli, int offsetco,
		       int init_Ivariation,
		       int *Nbli_ima_out, int *Nbcol_ima_out,
		       int *Offset_li, int *Offset_co, float *Ivariation,
		       TImageFloat *imFx, TImageFloat *imFy);

void Recale_image (unsigned char *src, unsigned char *dst,
		   int ysize, int xsize,
		   int *Nbli_ima_out, int *Nbcol_ima_out, float *Ivariation,
		   TImageFloat *imFx, TImageFloat *imFy);
void Recale_image(short *src, short *dst, int ysize, int xsize,
		  int *Nbli_ima_out, int *Nbcol_ima_out, float *Ivariation,
		  TImageFloat *imFx, TImageFloat *imFy);

void copie_sup (const TImageFloat *src, TImageShort *dst, const TWindow *win);


bool Actualise_deplacements_para(Para *param,
				 int *Nbli_ima_out, int *Nbcol_ima_out,
				 float *Ivariation,
				 TImageFloat *imFx, TImageFloat *imFy);

bool Back_Warp_ima(short *src, int ysize, int xsize, short *dst,
		   Para *param);


bool compose_para(Para *par1, Para *par2, Para *par_out);
bool masque_suivant(short *segm_in, int num_reg, short *segm_out,
		    int num_reg_out, int nbli, int nbco, Para *param);

void free_recalage_film(TImageFloat *imFx, TImageFloat *imFy);
void get_deplacement (int x, int y, float *xdepl, float *ydepl,
		      int *Nbcol_ima_out, TImageFloat *imFx, TImageFloat *imFy);
bool Warp_region(short *segm_in, int num_reg, int num_reg_out,
		 short *segm_out, int nbli, int nbco, Para *param);


/*
 * PROCEDURE	: Warp_image
 *
 * INPUTS      :
 * ima_in         Image a recaler.
 * nbli           Nombre de lignes de l'image d'entree a recaler.
 * nbco           Nombre de colonnes de l'image d'entree a recaler.
 *
 * OUTPUT       :
 * ima_out        Image de sortie recalee.
 *
 * INPUT       :
 * param          Coefficients du modele de mouvement. Ces coefficients
 *                representent le mouvement cumule depuis la premiere
 *                image de la sequence.
 * v_init_def	  Valeur des points qui n'ont pas d'antecedents.
 *
 * DESCRIPTION	:
 * La procedure determine l'image suivante en fonction des parametres de
 * mouvement (cas ou le nb de para est superieur a 6 parametres).
 */

template <class T>
bool Warp_image(T *src, T *dst, int ysize, int xsize, Para *param, T oval)
{
  double theta[13];
  float
    pond,
    *im_pond,
    *pt_pond,
    Iprec = 0;
  int i, j, x_, y_;
  double varalt=0, i_dbl, j_dbl;

  short *ptr_int=NULL,*ima_int;
  T *ptr, *ptr_ima_in;

  if (model_degree(param->id_model) < 2) {
    if (Warp_ima_6para(src, dst, ysize, xsize, param, oval) == false)
      return false;
  }
  else {
    /* Obtention des parametres dans le repere (0,0) */
    if (chgt_repere(param, theta, 0, 0) == false)
      return false;

    double u = theta[0];
    double v = theta[1];

    double a = theta[2];
    double b = theta[3];
    double c = theta[4];
    double d = theta[5];

    double q1 = theta[6];
    double q2 = theta[7];
    double q3 = theta[8];
    double q4 = theta[9];
    double q5 = theta[10];
    double q6 = theta[11];

    if  (param->var_light) varalt = param->thet[12];

    /* on veut les parametres de deplacement, pas de mouvement */
    a += 1.0;  d += 1.;
    /* Utilisation d'un masque intermediaire (si ima_in = ima_out) */
    /* calloc met la memoire a zero */
    ima_int = (short *)calloc((size_t)(ysize*xsize), sizeof(short));
    /* Utilisation d'une image de ponderation  */
    im_pond = (float *)calloc((unsigned)(ysize*xsize),sizeof(float));

    ptr_ima_in = src;
    for (j = 0; j < ysize;j++ ){
      j_dbl = (double) j;
      for (i = 0; i < xsize;i++,ptr_ima_in++){
	i_dbl = (double) i;

	double x =  u + i_dbl*(a+q1*i_dbl+q2*j_dbl)+j_dbl*(b+q3*j_dbl);
	double y =  v + i_dbl*(c+q4*i_dbl+q5*j_dbl)+j_dbl*(d+q6*j_dbl);

	/* on modifie tous les points autour de x,y qui sont dans l'image,
	 * en utilisant la ponderation. */
	x_ = (int)x; y_ = (int)y;
	if( INTERPLT2D_POSSIBLE(y_,x_,ysize,xsize)){
	  Iprec = (float)(*ptr_ima_in - varalt);
	  ptr_int = ima_int+y_*xsize+x_;
	  pt_pond = im_pond+y_*xsize+x_;
	  pond = (float)((x_+1.0-x)*(y_+1.0-y));
	  *ptr_int = (short)(pond * Iprec + *pt_pond * (float)(*ptr_int));
	  *pt_pond += pond;  //*ptr_int++ /= *pt_pond++;
	  // FS: Remplace le commentaire ci dessus.
	  *ptr_int = (short) (*ptr_int / *pt_pond);
	  ptr_int++; pt_pond++;

	  pond = (float)((x-x_)*(y_+1.0-y));
	  *ptr_int = (short)(pond * Iprec + *pt_pond * (float)(*ptr_int));
	  *pt_pond += pond; //*ptr_int /= *pt_pond;
	  // FS: Remplace le commentaire ci dessus.
	  *ptr_int = (short) (*ptr_int / *pt_pond);

	  ptr_int += xsize; pt_pond += xsize;
	  pond = (float)((x-x_)*(y-y_));
	  *ptr_int = (short)(pond * Iprec + *pt_pond * (float)(*ptr_int));
	  *pt_pond += pond; //*ptr_int-- /= *pt_pond--;
	  // FS: Remplace le commentaire ci dessus.
	  *ptr_int = (short) (*ptr_int / *pt_pond);
	  ptr_int--; pt_pond--;

	  pond = (float)((x_+1.-x)*(y-y_));
	  *ptr_int = (short)(pond * Iprec + *pt_pond * (float)(*ptr_int));
	  *pt_pond += pond; //*ptr_int /= *pt_pond;
	  // FS: Remplace le commentaire ci dessus.
	  *ptr_int = (short) (*ptr_int / *pt_pond);
	}
      }
    }
    /* Recopie de la region projetee dans l'image de sortie */
    ptr_int = ima_int; ptr = dst;
    for(i=0;i<ysize*xsize;i++,ptr++,ptr_int++)
      *ptr = (T) *ptr_int;

    free(ima_int); ima_int = NULL;
    free(im_pond); im_pond = NULL;
  }
  return true;
}

/*
 * PROCEDURE	: Warp_ima_6para
 *
 * INPUTS      :
 * ima_in         Image a recaler.
 * nbli           Nombre de lignes de l'image d'entree a recaler.
 * nbco           Nombre de colonnes de l'image d'entree a recaler.
 *
 * OUTPUT       :
 * ima_out        Image de sortie recalee.
 *
 * INPUT       :
 * param          Coefficients du modele de mouvement. Ces coefficients
 *                representent le mouvement cumule depuis la premiere
 *                image de la sequence.
 * v_init_def	  Valeur des points qui n'ont pas d'antecedents.
 *
 * DESCRIPTION	:
 * La procedure determine l'image suivante en fonction des parametres de
 * mouvement (cas ou le nb de para est egal a 6 parametres).
 */
template <class T>
bool Warp_ima_6para(T *src, T *dst, int ysize, int xsize, Para *param, T oval)
{
#define	BYTEMIN	0
#define	BYTEMAX	255

#define  EPSILON  1.E-10

  double theta[13];
  double u,v;              /* translation vector */
  double a, b, c, d;      /* matrix coefficients */
  double delta;            /* matrix determinant */
  double u_,  v_;          /* inverse  translational coeficient */
  double a_, b_, c_,  d_;  /* inverse  matrix coefficients */
  double di = 0.; // Illumination variation

  T *_dst = new T [xsize*ysize]; // temporary map if src = dst
  T *_ptdst = _dst;

  memset(_dst, oval, xsize*ysize*sizeof(T));

  /* computation of the inverse transform */
  /* Obtention des parametres dans le repere (0,0) */
  if (chgt_repere(param,theta,0,0) == false)
    return false;

  u = theta[0];  v = theta[1];
  a = theta[2];  b = theta[3];
  c = theta[4];  d = theta[5];

  a += 1.0;  d+= 1.0; /* on veut le champ des deplacements, pas des vitesses */

  delta = a*d - b*c;

  if ((-EPSILON < delta) && (delta < EPSILON)) {
    printf("\nCan not warp image...\n");
    return false;
  }

  /* coefficients of the inverse transform */
  a_ =  d /delta;      b_ = - b /delta;        u_ = (b*v - d*u)/delta;
  c_ = -c /delta;      d_ =   a /delta;        v_ = (c*u - a*v)/delta;

  /* coefficients of the direct speed transform */
  a -= 1.0;	d -= 1.0;

  if  (param->var_light) di = param->thet[12];

  for (int y = 0; y < ysize; y++ )  {
    for (int x = 0; x < xsize; x++) {

      double x_dbl = (double) x;
      double y_dbl = (double) y;

      double fx = a_ * x_dbl + b_ * y_dbl + u_;
      double fy = c_ * x_dbl + d_ * y_dbl + v_;

#if 1 // Original version
      INTERPLT2D_DECLARE(xsize, fx, fy);
      /* test for x,y in the image */
      if (INTERPLT2D_ISIN(xsize, ysize))  {
	double val;           /* valeur interpolee */
	double s;

	INTERPLT2D_PROCESS(T, (T *) src, val);

	s = val - di;
	if (s < BYTEMIN)	s = BYTEMIN;
	else if (s > BYTEMAX)	s = BYTEMAX;
	*_ptdst = (T) s;
      }
#else
      int	_x0  = (int) (fx);
      int	_y0  = (int) (fy);
      if (x < 0.f) _x0 = -1;
      if (y < 0.f) _y0 = -1;
      size_t	_n   = (size_t) (xsize);
      size_t	_off = MIJ(0, _y0, _x0, _n);
      double	_dx  = (fx) - (double) _x0;
      double	_dy  = (fy) - (double) _y0;
      double	_v01;
      double	_v23;

      /* test for x,y in the image */
      if ((0 <= _x0) && (_x0 < ((xsize) - 1)) && \
	  (0 <= _y0) && (_y0 < ((ysize) - 1)))  {
	double val;           /* valeur interpolee */
	double s;

	{
	  T	*_mp = &((T *) src)[_off];
	  _v01 = (double) _mp[0] + (_dx * (double) (_mp[1] - _mp[0]));
	  _mp += _n;
	  _v23 = (double) _mp[0] + (_dx * (double) (_mp[1] - _mp[0]));
	  val = _v01 + (_dy * (_v23 - _v01));
	}

	s = val - di;
	if (s < BYTEMIN)	s = BYTEMIN;
	else if (s > BYTEMAX)	s = BYTEMAX;
	*_ptdst = (T) s;
      }
#endif
      else  {
	/* x,y not in the image */
	*_ptdst = oval;
      }
      _ptdst++;
    }
  }

  memcpy(dst, _dst, xsize*ysize*sizeof(T));

#undef EPSILON
#undef BYTEMIN
#undef BYTEMAX
  delete [] _dst;
  return true;
}


#endif   /* compense_h  */
