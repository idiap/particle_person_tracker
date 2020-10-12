/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include	<float.h>
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>

#include	"invert.h"


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
#define	MIJ(m,i,j,s)	((m) + ((long) (i) * (long) (s)) + (long) (j))


/*
 * PROCEDURE    : dgedi
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * a		Pointeur sur la matrice obtenue par dgeco ou dgefa.
 * lda		Nombre d'elements maximum dans une ligne de la matrice "a"
 * n		Ordre de la matrice "a".
 * ipvt		Vecteur des pivots de dgeco ou de dgefa.
 * work		Vecteur de travail dont le contenu est detruit.
 * job		= 11 recherche de l'inverse et du determinant.
 *		= 01 recherche de l'inverse seulement.
 *		= 10 recherche du determinant seulement.
 *
 * OUTPUT	:
 * a		Inverse de la matrice originale si demandee, sinon inchangee.
 * det		determinant de la matrice originale si demandee, sinon non
 *		referencee.
 *
 *				                         det[1]
 *				determinant = det[0] * 10
 *
 *			avec 1.0 <= fabs(det[0]) < 10.0 ou det[0] = 0.0
 *
 * DESCRIPTION  :
 * La procedure calcule le determinant "det" et l'inverse de la matrice "a"
 * d'ordre "n" en utilisant les facteurs calcules par les procedures dgeco
 * ou dgefa.
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * CONDITION D'ERREUR
 *
 * Une division par zero apparaitra si un des elements de la diagonale de la
 * matrice "buf" est nul et que l'inverse est demande. Elle n'apparaitra pas
 * les procedures sont appelees correctement et la variable "rcond" de dgeco
 * est > 0.0 ou la variable "info" de dgefa est initialisee a 0.0.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Cleve Moler
 * 1.01 - 29/05/95 - Amelioration et correction de bugs.
 */
int	dgedi (double *a, size_t lda, size_t n, int *ipvt, double *det,
	       double *work, int job)
{
  double	tmp;			/* Variable temporaire */
  int		i, j, k, kb, kp1, nm1;	/* Compteur de boucle */
  int		ip;			/* Indice de l'element pivot	*/

  /* Calcul du determinant	*/
  if (job / 10 != 0)
    {
      det[0] = 1.0;
      det[1] = 0.0;

      for (i = 0; i < (int) n; i++)
	{
	  if (ipvt[i] != i)
	    det [0] = -det[0];
	  det[0] *= *MIJ(a, i, i, lda);

	  /* sortie	*/
	  if (det[0] != 0.0)
	    {
	      while (fabs (det[0]) < 1.0)
		{
		  det[0] *= 10.0;
		  det[1] -= 1.0;
		}

	      while (fabs (det[0]) >= 10.0)
		{
		  det[0] /= 10.0;
		  det[1] += 1.0;
		}
	    }
	}
    }

  /* Calcul de inverse (u)	*/
  if ((job % 10) != 0)
    {
      for (k = 0; k < (int) n; k++)
	{
	  *MIJ(a, k, k, lda) = 1.0 / *MIJ(a, k, k, lda);
	  tmp = -*MIJ(a, k, k, lda);
	  dscal (MIJ(a, k, 0, lda), (size_t) k, tmp, 1);

	  kp1 = k + 1;

	  if ((int) n >= kp1)
	    {
	      for (i = kp1; i < (int) n; i++)
		{
		  tmp = *MIJ(a, i, k, lda);
		  *MIJ(a, i, k, lda) = 0.0;
		  daxpy (MIJ(a, i, 0, lda), MIJ(a, k, 0, lda), (size_t) (k + 1),
			 tmp, 1, 1);
		}
	    }
	}
      /* calcul de inverse (u) * inverse (l)	*/
      nm1 = (int) n - 1;
      if (nm1 >= 0)
	{
	  for (kb = 0; kb < nm1; kb++)
	    {
	      k = (int) n - kb - 2;

	      kp1 = k + 1;

	      for (j = kp1; j < (int) n; j++)
		{
		  work[j] = *MIJ(a, k, j, lda);
		  *MIJ(a, k, j, lda) = 0.0;
		}

	      for (i = kp1; i < (int) n; i++)
		{
		  tmp = work[i];
		  daxpy (MIJ(a, k, 0, lda), MIJ(a, i, 0, lda), n, tmp, 1, 1);
		}

	      ip = ipvt[k];

	      if (ip != k)

		dswap(MIJ(a, k, 0, lda), MIJ(a, ip, 0, lda), n, 1, 1);
	    }
	}
    }
  return (0);
}

/*
 * PROCEDURE    : dgeco
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * a		Pointeur sur la matrice a factoriser.
 * lda		Nombre d'elements maximum dans une ligne de la matrice "a"
 * n		Ordre de la matrice "a".
 *
 * OUTPUT	:
 * a		Matrice triangulaire superieure et les multiplicateurs
 *		qui ont permis son obtention.
 *		La factorisation peut etre ecrite A = L x U ou L est le produit
 *		des matrices de permutation et triangulaire inferieure unitaire
 *		et U est triangulaire superieure.
 * ipvt		Vecteur contenant les indices des pivots.
 * rcond	Estimation de l'inverse du nombre de conditionnement de "a".
 *		Pour le systeme A x X = B, des perturbations relatives de
 *		l'ordre d'epsilon dans les valeurs de "A" ou "B" peuvent
 *		causer des perturbations dans les elements de "X" de l'ordre
 *		de epsilon / rcond.
 *		Si rcond est tres petit tel que l'expression logique
 *			1.0 + rcond = 1.0
 *		est vrai, alors "a" peut etre singuliere, en particulier
 *		rcond = 0.0 si une singularite exacte est detectee ou si
 *		l'estimation depasse la limite inferieure.
 * z		Vecteur de travail. Son contenu n'est pas en general important.
 *		Si "a" est proche d'une matrice singuliere, alors z est
 *		approximativement un vecteur nul au sens de
 *		norm (a * z) = rcond * norm (a) * norm (z).
 *
 * DESCRIPTION  :
 * La procedure factorise la matrice "a" d'ordre "n" de reels double
 * precision par l'elimination de Gauss et estime le nombre de conditionnement
 * "rcond" de la matrice "a".
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Cleve Moler
 * 1.01 - 29/05/95 - Amelioration et correction de bugs.
 */
int	dgeco (double *a, size_t lda, size_t n, int *ipvt,
	       double *rcond, double *z)
{
  double	ek;
  double	tmp;		/* Variable temporaire	*/
  double	wk, wkm;        /* Variables de travail	*/

  double	anorm;		/* norme matricielle	*/
  double	s;
  double	sm;
  double	ynorm;

  int		infor;
  int		i, k, kb, kp1;	/* compteur de boucle	*/
  int		ip;		/* indice de l'element pris comme pivot	*/

  /* Recherche de la norme matricielle maximum.
   * REMARQUE : la norme matricielle utilisee est ||a||1 appliquee sur
   * les colonnes de la matrice.
   */
  anorm = 0.0;

  for (i = 0; i < (int) n; i++)
    anorm = ( anorm > dasum (MIJ(a, i, 0, lda), n, 1)) ? anorm : dasum(MIJ(a, i, 0, lda), n, 1);

  /* factorisation de Gauss	*/
  dgefa (a, lda, n, ipvt, &infor);

  /* rcond = 1 / (norm (A) * (estimation de norme (inverse (A)))).
   * estimation = norm (Z) / norm (Y) ou A x Z = Y et tran (A) * Y = E.
   * trans (A) est la transposee de A. Les composants de e sont choisis
   * de telle facon qu'ils causent une croissance locale maximale dans
   * les elements de W ou trans (U) * W = E. Les vecteurs sont generalement
   * mis a l'echelle afin d'eviter l'overflow.
   */

  /* Resolution de trans (u) * w = e.	*/
  ek = 1.0;

  for (k = 0; k < (int) n; k++)
    z[k] = 0.0;

  for (k = 0; k < (int) n; k++)
    {
      if (z[k] != 0.0)
	ek = dsign (ek, -z[k]);

      if (fabs (ek - z[k]) > fabs (*MIJ(a, k, k, lda)))
	{
	  s = fabs (*MIJ(a, k, k, lda)) / fabs (ek - z[k]);
	  dscal (z, n, s, 1);
	  ek *= s;
	}

      wk = ek - z[k];
      wkm = - ek - z[k];
      s = fabs (wk);
      sm = fabs (wkm);

      if (*MIJ(a, k, k, lda) == 0.0)
	{
	  wk = 1.0;
	  wkm = 1.0;
	}
      else
	{
	  wk /= *MIJ(a, k, k, lda);
	  wkm /= *MIJ(a, k, k, lda);
	}

      kp1 = k + 1;
      if (kp1 < (int) n)
	{
	  for (i = kp1; i < (int) n; i++)
	    {
	      sm += fabs (z[i] + wkm * *MIJ(a, i, k, lda));
	      z[i] += wk * *MIJ(a, i, k, lda);
	      s += fabs (z[i]);
	    }

	  if ( s < sm)
	    {
	      tmp = wkm - wk;
	      wk = wkm;

	      for (i = kp1; i < (int) n; i++)
		z[i] += tmp * *MIJ(a, i, k, lda);
	    }
	}
      z[k] = wk;
    }

  s = 1.0 / dasum (z, n, 1);
  dscal (z, n, s, 1);

  /* resolution de trans (l) * y = w	*/
  for (kb = 0; kb < (int) n; kb++)
    {
      k = (int) n - kb - 1;

      if ( k < (int) n - 1)
	z[k] += ddot (MIJ(a, k, k + 1, lda), &z[k + 1],
		      n - k - 1, 1, 1);
      if ( abs (z[k] > 1.0))
	{
	  s = 1.0 / fabs (z[k]);
	  dscal (z, n, s, 1);
	}

      ip = ipvt[k];

      SWAP(z[ip], z[k], tmp);
    }

  s = 1.0 / dasum (z, n, 1);
  dscal (z, n, s, 1);

  ynorm = 1.0;

  /* Resolution de l * v = y	*/

  for (k = 0; k < (int) n; k++)
    {
      ip = ipvt[k];
      SWAP(z[ip], z[k], tmp);

      if (k < (int) n)
	daxpy (&z[k + 1], MIJ(a, k, k + 1, lda), ((size_t) n - k - 1),
	       tmp, 1, 1);
      if (fabs (z[k]) > 1.0)
	{
	  s = 1.0 / fabs (z[k]);
	  dscal (z, n, s, 1);
	  ynorm *= s;
	}
    }

  s = 1.0 / dasum (z, n, 1);
  dscal (z, n, s, 1);
  ynorm *= s;

  /* Resolution de u * z = v	*/
  for ( kb = 0; kb < (int) n; kb++)
    {
      k = (int) n - kb - 1;

      if ( fabs (z[k]) > fabs (*MIJ(a, k, k, lda)))
	{
	  s = fabs (*MIJ(a, k, k, lda)) / fabs (z[k]);
	  dscal (z, n, s, 1);
	  ynorm *= s;
	}

      if (*MIJ(a, k, k, lda) != 0.0)
	z[k] = z[k] / *MIJ(a, k, k, lda);

      else if (*MIJ(a, k, k, lda) == 0.0)
	z[k] = 1.0;

      tmp = - z[k];
      daxpy (z, MIJ(a, k, 0, lda), (size_t) k, tmp, 1, 1);
    }

  /* normalisation de znorm = 1	*/
  s = 1.0 / dasum (z, n, 1);
  dscal (z, n, s, 1);

  ynorm *= s;

  if (anorm != 0.0)
    *rcond = ynorm / anorm;

  else if (anorm == 0.0)
    *rcond = 0.0;

  return (0);
}


/*
 * PROCEDURE    : dgefa
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * a		Pointeur sur la matrice a factoriser.
 * lda		Nombre d'elements maximum dans une ligne de la matrice "a"
 * n		Ordre de la matrice "a".
 *
 * OUTPUT	:
 * a		Matrice contenant la triangulaire superieure et les multipli-
 *		cateurs qui ont permis l'obtention de cette matrice
 *		triangulaire.
 *		La factorisation peut etre ecrite A = L x U ou L est le produit
 *		des matrices de permutation et est triangulaire inferieure
 *		unitaireet U est triangulaire superieure.
 * ipvt		Vecteur contenant les indices des pivots.
 * info		= 50 en cas de succes
 *		= k si u[k][k] = 0. Ce n'est pas une erreur pour cette
 *		procedure mais elle indique que dgedi divisera par zero en
 *		cas d'appel. Il vaut mieux utiliser rcond de dgeco pour une
 *		indication fiable de la singularite.
 *
 * DESCRIPTION  :
 * La procedure factorise la matrice "a" de taille "raw" x "col" de reels
 * double precision par l'elimination de Gauss.
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Cleve Moler
 * 1.01 - 29/05/95 - Amelioration et correction de bugs.
 */
int	dgefa (double *a, size_t lda, size_t n, int *ipvt, int *info)
{
  double	tmp;		/* variable temporaire	*/
  int		ip;		/* indice de l'element pris comme pivot	*/
  int		i, k, nm1, kp1;	/* compteurs de boucle	*/

  /* Elimination de Gauss avec pivotage partiel	*/
  *info = 0;
  nm1 = (int) n - 1;

  /* cas ou la matrice ne contient qu'un seul element	*/
  if (nm1 == 0)
    {
      ipvt [nm1] = nm1;

      if (*MIJ(a, nm1, 0, lda) == 0.0)
	*info = (int) (n) - 1;

      return (50);
    }

  if (nm1 < 0)
    {
      perror ("matrice de taille nullz");
      return (-1);
    }

  for (k = 0; k < nm1; k++)
    {

      kp1 = k + 1;
      /* recherche du pivot dans la ligne dont l'element est le
       * plus grand en valeur absolue et le plus proche de l'element
       * a[k][k]
       */
      ip = idamax (MIJ(a, k, k, lda), ((size_t) n - k), 1) + k;
      ipvt [k] = ip;

      /* Rem: pas de pivot implique que cette ligne est deja
       * triangularisee.
       */
      if (*MIJ(a, k, k, lda) != 0.0)
	{

	  /* Interchangement si necessaire	*/
	  if (ip != k)
	    SWAP(*MIJ(a, k, ip, lda), *MIJ(a, k, k, lda), tmp);

	  /* Calcul des multiplicateurs	*/
	  tmp = -1.0 / *MIJ(a, k, k, lda);
	  dscal (MIJ(a, k, k + 1, lda), ((size_t) n - k - 1),tmp, 1);

	  /* Elimination	*/
	  for (i = kp1; i < (int) n; i++)
	    {
	      tmp = *MIJ(a, i, ip, lda);

	      if (ip != k)
		SWAP(*MIJ(a, i, ip, lda),
		     *MIJ(a, i, k, lda), tmp);

	      daxpy (MIJ(a, i, k + 1, lda), MIJ(a, k, k + 1, lda),
		     ((size_t) n - k - 1), tmp, 1, 1);
	    }
	}
      else
	*info = k ;
    }


  ipvt[(int) n - 1] = (int) (n) - 1;

  if (*MIJ(a, n - 1, n - 1, lda) == 0.0)
    *info = (int) (n) - 1;

  return (50);
}


#ifdef	INVERT_MAIN

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>


#define	RAW	9
#define	COL	RAW


/*
 * PROCEDURE    : main
 *
 * DESCRIPTION  :
 * La procedure teste les procedure dgefa, dgeco et dgedi.
 *
 * HISTORIQUE   :
 * 1.00 - 01/06/95 - Original.
 */
void main (void)
{
#if 0
  double matrix[RAW][COL] = {{1., 5., 8., 4.},
			     {5., 3., 1., 4.},
			     {8., 8., 8., 8.},
			     {1., 4., 6., 1.}};
#endif
  double matrix[RAW][COL] =
  {{ 5.4125999988e6, 6.2795590855e5,-1.6436440871e6, 0.,             0.,             0.,             2.8425692480e3, 0.,             1.5837442351e6},
   { 6.2795590855e5, 6.5215970159e6, 2.1429000032e6, 0.,             0.,             0.,            -9.1823095520e3, 0.,             7.7995847656e6},
   {-1.6436440871e6, 2.1429000032e6, 1.6261002981e7, 0.,             0.,             0.,            -4.9317690454e4, 0.,             1.8391879506e7},
   { 0.,             0.,             0.,             5.4125999988e6, 6.2795590855e5,-1.6436440871e6, 0.,             2.8425692480e3,-3.6401126773e6},
   { 0.,             0.,             0.,             6.2795590855e5, 6.5215970159e6, 2.1429000032e6, 0.,            -9.1823095520e3, 4.0465530928e6},
   { 0.,             0.,             0.,            -1.6436440871e6, 2.1429000032e6, 1.6261002981e7, 0.,            -4.9317690454e4, 1.2945310731e7},
   { 2.8425692480e3,-9.1823095520e3,-4.9317690454e4, 0.,             0.,             0.,             1.7000000000e2, 0.,            -6.6545601487e4},
   { 0.,             0.,             0.,             2.8425692480e3,-9.1823095520e3,-4.9317690454e4, 0.,             1.7000000000e2,-4.4258337967e4},
   { 1.5837442351e6, 7.7995847656e6, 1.8391879506e7,-3.6401126773e6, 4.0465530928e6, 1.2945310731e7,-6.6545601487e4,-4.4258337967e4, 4.3830755320e7}};

  double	det[2], array [RAW][COL], test[RAW][COL];
  int		ipvt[RAW];
  double	rcond, s, z[RAW];
  int		info;
  int		job = 11;
  int		i, j, k;

  printf ("XMATRIX =\n");
  show_mtx (&matrix[0][0], RAW, COL);
  /* recopie matrice dans array	*/
  for (i = 0; i < RAW; i++)
    {
      for (j = 0; j < COL; j++)
	array[i][j] = matrix[i][j];
    }
#if 0

  dgefa (&array[0][0], RAW, COL, ipvt, &info);
  printf ("\n\n\nMATRICE FACTORISEE =\n");
  show_mtx (&array[0][0], RAW, COL);

  for (i = 0; i <RAW; i++)
    printf ("ivpvt[%d] = %d\n",i, ipvt[i]);

  printf ("info =%d\n",info);
#endif

  dgeco (&matrix[0][0], RAW, COL, ipvt, &rcond, z);

  printf ("apres elimination\n");
  printf ("XMATRIX =\n");
  show_mtx (&matrix[0][0], RAW, COL);
  printf ("vecteur des indices des pivots\n");
  for (i = 0; i < RAW; i++)
    printf ("ipvt[%d] =%d\n",i, ipvt[i]);

  printf ("rcond =%5.10e\n", rcond);
  for (i = 0; i < RAW; i++)
    printf ("z[%d] = %5.10e\n", i, z[i]);

  printf ("job avt entree dans proc =%d\n",job);
  dgedi (&matrix[0][0], RAW, COL, ipvt, det, z, job);

  printf ("det =%lf 10^%lf\n", det[0], det[1]);

  for (i = 0; i < 2; i++)
    printf ("det[%d] = %lf\n", i, det[i]);
  printf ("matrix inverse :\n");
  show_mtx (&matrix[0][0], RAW, COL);
  for (i = 0; i < RAW; i++)
    printf ("z[%d] = %5.10e\n", i, z[i]);

  printf("preuve A^-1 * A = I\n");
  for (i = 0; i < RAW; i++)
    {
      for (j = 0; j < COL; j++)
	{
	  s = 0.0;
	  for (k = 0; k < COL; k++)
	    s += array[i][k] * matrix[k][j];

	  if (fabs(s) <= 1e-10)
	    s = 0.;
	  test[i][j] = s;
	}
    }
  printf ("test :\n");
  show_mtx (&test[0][0], RAW, COL);
}

#endif

