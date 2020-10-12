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
 DESCRIPTION	: Le fichier contient les macro-instructions d'interpolation.

 RESUME		:

 MIJ			Acces a un element d'une matrice.
 INTERPLT2D_TYPE	Interpole la valeur d'un pixel dans une matrice.

 INTERPLT2D_DECLARE	Declare et precalcule une interpolation bilineaire.
 INTERPLT2D_ISIN	Teste l'appartenance d'un point dans une image.
 INTERPLT2D_PROCESS	Execute une interpolation bilineaire precalculee.

 HISTORIQUE	:

 1.00 - 01/07/95 - Original.

*/


#ifndef	__INTERPLT_H
#define	__INTERPLT_H


#include	<stdlib.h>	/* pour size_t	*/

/*
 * MACRO	: MIJ
 *
 * INPUT	:
 * m		Matrice.
 * i		Indice ligne   de l'element.
 * j		Indice colonne de l'element.
 * s		Taille en nombre d'elements d'une ligne de la matrice "m".
 *
 * DESCRIPTION	:
 * La macro-instruction calcule l'adresse de l'element de la "i"eme ligne et
 * de la "j"eme colonne de la matrice "m", soit &m[i][j].
 *
 * RETOUR	:
 * L'adresse de m[i][j] est retournee.
 *
 * HISTORIQUE	:
 * 1.00 - 11/02/93 - Original.
 */
#define	MIJ(m,i,j,s)	((m) + ((i) * (s)) + (j))

/*
 * MACRO	: INTERPLT2D_TYPE
 *
 * INPUT	:
 * t		Type d'un element de la matrice.
 * m		Matrice.
 * xsize	Largeur de l'image en pixels.
 * x		Indice de colonne dans l'image.
 * y		Indice de ligne   dans l'image.
 *
 * OUTPUT	:
 * v		Valeur interpolee.
 *
 * DESCRIPTION	:
 * La macro-instruction interpole la valeur d'un pixel dans la matrice "m"
 * d'elements de type "t" et de largeur "xsize" en pixels. La macro-instruction
 * interpole de maniere bilineaire les valeurs des 4 pixels voisins du pixel de
 * coordonnees ("x","y"). La valeur resultat est stockee dans le parametre "v"
 * de type float.
 *
 * HISTORIQUE	:
 * 1.00 - 26/06/95 - Original.
 */
#define	INTERPLT2D_TYPE(t,m,xsize,x,y,v)	\
{\
	int	_x0  = (int) (x);\
	int	_y0  = (int) (y);\
	t	*_mp = MIJ(m, _y0, _x0, xsize);\
	float	_dx  = (x) - (float) _x0;\
	float	_dy  = (y) - (float) _y0;\
	float	_v01;\
	float	_v23;\
\
	_v01 = (float) _mp[0] + (_dx * (float) (_mp[1] - _mp[0]));\
	_mp += (xsize);\
	_v23 = (float) _mp[0] + (_dx * (float) (_mp[1] - _mp[0]));\
	(v) = _v01 + (_dy * (_v23 - _v01));\
}

/*
 * MACRO	: INTERPLT2D_X
 *
 * INPUT	:
 * m		Matrice.
 * xsize	Largeur de l'image en pixels.
 * x		Indice de colonne dans l'image.
 * y		Indice de ligne   dans l'image.
 *
 * OUTPUT	:
 * v		Valeur interpolee.
 *
 * DESCRIPTION	:
 * La macro-instruction interpole la valeur d'un pixel dans la matrice "m"
 * d'elements de type X et de largeur "xsize" en pixels. La macro-instruction
 * interpole de maniere bilineaire les valeurs des 4 pixels voisins du pixel
 * de coordonnees ("x","y"). La valeur resultat est stockee dans le parametre
 * "v" de type float.
 *
 * HISTORIQUE	:
 * 1.00 - 26/06/95 - Original.
 */
#define	INTERPLT2D_FLOAT(m,xsize,x,y,v)	INTERPLT2D_TYPE(float,m,xsize,x,y,v)
#define	INTERPLT2D_SHORT(m,xsize,x,y,v)	INTERPLT2D_TYPE(short,m,xsize,x,y,v)

/*
 * MACRO	: INTERPLT2D_DECLARE
 *
 * INPUT	:
 * xsize	Largeur d'une image en pixels.
 * x		Indice de colonne dans une image.
 * y		Indice de ligne   dans une image.
 *
 * DESCRIPTION	:
 * La macro-instruction declare et precalcule les variables pour
 * l'interpolation bilineaire de la valeur d'un pixel dans une matrice.
 *
 * REMARQUE	:
 * La macro-instruction doit etre precedee d'une ouverture de bloc '{'.
 *
 * HISTORIQUE	:
 * 1.00 - 04/07/95 - Original.
 */
#define	INTERPLT2D_DECLARE(xsize,x,y)	\
	int	_x0  = (int) (x);\
	int	_y0  = (int) (y);\
	if (x < 0.f) _x0 = -1;\
	if (y < 0.f) _y0 = -1;\
	size_t	_n   = (size_t) (xsize);\
	size_t	_off = MIJ(0, _y0, _x0, _n);\
	double	_dx  = (x) - (double) _x0;\
	double	_dy  = (y) - (double) _y0;\
	double	_v01;\
	double	_v23;

/*
 * MACRO	: INTERPLT2D_ISIN
 *
 * GLOBAL	:
 * _x0		Coordonnee horizontale entiere.
 * _y0		Coordonnee verticale   entiere.
 *
 * INPUT	:
 * xsize	Largeur d'une image en pixels.
 * ysize	Hauteur d'une image en pixels.
 * x		Indice de colonne dans une image.
 * y		Indice de ligne   dans une image.
 *
 * DESCRIPTION	:
 * La macro-instruction teste l'appartenance du point "(_x0, _y0)" dans une
 * image de taille "xsize" x "ysize" afin d'interpoler bilineairement le point.
 *
 * HISTORIQUE	:
 * 1.00 - 04/07/95 - Original.
 */
#define	INTERPLT2D_ISIN(xsize,ysize)	\
	((0 <= _x0) && (_x0 < ((xsize) - 1)) && \
	 (0 <= _y0) && (_y0 < ((ysize) - 1)))

/*
 * MACRO	: INTERPLT2D_PROCESS
 *
 * GLOBAL	:
 * _off		Decalage dans la matrice.
 * _dx		Distance horizontale.
 * _dy		Distance verticale.
 * _v01		Composante de ligne.
 * _v23		Composante de ligne suivante.
 *
 * INPUT	:
 * t		Type d'un element de la matrice.
 * m		Matrice.
 *
 * OUTPUT	:
 * v		Valeur interpolee.
 *
 * DESCRIPTION	:
 * La macro-instruction interpole la valeur d'un pixel dans la matrice "m"
 * d'elements de type "t". La macro-instruction interpole de maniere bilineaire
 * les valeurs des 4 pixels voisins du pixel de coordonnees definies par
 * INTERPLT2D_PROCESS. La valeur resultat est stockee dans le parametre "v" de
 * type float.
 *
 * REMARQUE	:
 * La macro-instruction doit etre precedee d'un appel a INTERPLT2D_PROCESS.
 *
 * HISTORIQUE	:
 * 1.00 - 04/07/95 - Original.
 */
#define	INTERPLT2D_PROCESS(t,m,v)	\
{\
	t	*_mp = &(m)[_off];\
\
	_v01 = (double) _mp[0] + (_dx * (double) (_mp[1] - _mp[0]));\
	_mp += _n;\
	_v23 = (double) _mp[0] + (_dx * (double) (_mp[1] - _mp[0]));\
	(v) = _v01 + (_dy * (_v23 - _v01));\
}

#define INTERPLT2D_POSSIBLE(li,co,nbli,nbco) ((((int)li) >= 0) && (((int)co) >= 0) && \
         (((int)li) < (nbli-1)) && \
         (((int)co) < (nbco-1)))

#endif	/* __INTERPLT_H	*/

