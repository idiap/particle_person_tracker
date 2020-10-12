/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/


#include	<errno.h>
#include	<float.h>
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>

#include	"mtx_tool.h"


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
 * PROCEDURE    : dasum
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * buf		Pointeur sur un tableau de doubles.
 * count	Taille du tableau "buf".
 * incx		Increment.
 *
 * DESCRIPTION  :
 * La procedure calcule la somme des valeurs absolue des elements du tableau
 * "buf" de taille "count".Elle permet la sommation en utilisant des pas
 * de "incx".
 *
 * RETOUR	:
 * En cas de succes, la somme est retournee.
 * Sinon, la valeur DBL_MIN est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
double	dasum (double *buf, size_t count, int incx)
{
  double	dsum = 0.0;
  int	i;		/* compteur de boucle		*/
  int	m;		/* reste de la division entiere	*/

  if ((count == 0) || (incx == 0))
  {
    errno = EINVAL;
    return (DBL_MIN);
  }

  /* code dans le cas ou l'increment est different de 1	*/
  if (incx != 1)
  {
    double	*pc;
    double	*pend;

    if (incx > 1)
    {
      for (pc = buf, pend = buf + count; pc < pend ; pc += incx)
	dsum += fabs (*pc);

      return (dsum);
    }

    if (incx < 0)
    {
      for (pc = buf + count - 1, pend = buf; pc >= pend; pc += incx)
	dsum += fabs (*pc);

      return (dsum);
    }
  }

  /* code pour increment = 1	*/
  m = count % 6;

  if (m != 0)
  {
    for (i = 0; i < m; i++)
      dsum += fabs (buf[i]);

    if ( count < 6 )
      return (dsum);
  }

  for (i = m; i < (int) count; i += 6)
    dsum += fabs (buf[i]) + fabs (buf[i + 1]) + fabs (buf[i + 2])
      + fabs (buf[i + 3]) + fabs (buf[i + 4]) + fabs (buf[i + 5]);

  return (dsum);
}


/*
 * PROCEDURE    : daxpy
 *
 * GLOBAL	:
 * errno	Code d'erreur.
 *
 * INPUT	:
 * buf1		Pointeur sur un tableau de doubles.
 * buf2		Pointeur sur un tableau de doubles.
 * count	Taille des tableaux "buf1" et "buf2".
 * cst		Multiplicateur.
 * inc1		Increment dans le tableau "buf1".
 * inc2         Increment dans le tableau "buf1".
 *
 * DESCRIPTION  :
 * La procedure multiplie les elements du tableau "buf2" de taille "count"
 * par une constante "cst" et les additionne aux elements du tableau "buf1"
 * de taille "count". Elle permet le calcul en utilisant des pas de inc1 dans
 * le tableau "buf1" et des pas inc2 dans le tableau "buf2".
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
int daxpy (double *buf1, double *buf2, size_t count, double cst, int inc1, int inc2)
{
  int	i;		/* compteur de boucle			*/
  int	m;		/* reste de la division entiere		*/

  if ((count == 0) || (inc1 == 0) || (inc2 == 0) || (cst == 0))
  {
   errno = EINVAL;
   return (-1);
  }

  /*
   * code dans le cas ou les increments sont differents entre eux ou les
   * increments sont egaux mais differents de 1
   */
  if (inc1 != 1 && inc2 != 1)
  {
    int	i1, i2;		/* increments dans les deux tableaux	*/

    if (inc1 > 1 && inc2 > 1)
    {
      for (i1 = 0, i2 = 0; i1 < (int) count  && i2 < (int) count; i1 += inc1,
	     i2 += inc2)
	buf1[i1] += cst * buf2[i2];

      return (0);
    }

    if (inc1 > 1 && inc2 < 0)
    {
      for (i1 = 0, i2 = (int) count - 1; i1 < (int) count && i2 >= 0; i1 += inc1,
	     i2 += inc2)
	buf1[i1] += cst * buf2[i2];

      return (0);
    }

    if (inc1 < 0 && inc2 > 1)
    {
      for (i1 = (int) count - 1, i2 = 0; i1 >= 0 && i2 < (int) count; i1 += inc1,
	     i2 += inc2)
	buf1[i1] += cst * buf2[i2];

      return (0);
    }

    if (inc1 < 0 && inc2 < 0)
    {
      for (i1 = count - 1, i2 = count - 1; i1 >= 0 && i2 >= 0; i1 += inc1,
	     i2 += inc2)
	buf1[i1] += cst * buf2[i2];

      return (0);
    }
  }

  /* code pour inc1 et inc2 egaux a un	*/
  m = count % 4;

  if (m != 0)
  {
    for (i = 0; i < m; i++)
      buf1[i] += cst * buf2[i];

    if ( count < 4)
      return (0);
  }

  for (i = m; i < (int) count; i += 4)
  {
    buf1[i] += cst * buf2[i];
    buf1[i + 1] += cst * buf2[i + 1];
    buf1[i + 2] += cst * buf2[i + 2];
    buf1[i + 3] += cst * buf2[i + 3];
  }
  return (0);
}


/*
 * PROCEDURE    : ddot
 *
 * GLOBAL	:
 * errno	Code d'erreur.
 *
 * INPUT	:
 * buf1		Pointeur sur un tableau de doubles.
 * buf2		Pointeur sur un tableau de doubles.
 * count	Taille des tableaux "buf1" et "buf2".
 * inc1		Increment dans le tableau "buf1".
 * inc2         Increment dans le tableau "buf1".
 *
 * DESCRIPTION  :
 * La procedure calcule le produit scalaire du tableau "buf1" de taille "count"
 * par le tableau "buf2" de taille "count". Elle permet le calcul en utilisant
 * des pas de inc1 dans le tableau "buf1" et des pas inc2 dans le tableau "buf2".
 *
 * RETOUR	:
 * En cas de succes, le produit scalaire est retourne .
 * Sinon, la valeur DBL_MIN est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
double	ddot(double *buf1, double *buf2, size_t count, int inc1, int inc2)
{
  double	dot = 0.0;
  int	i;		/* compteur de boucle		*/
  int	m;		/* reste de la division entiere	*/

  if ((count == 0) || (inc1 == 0) || (inc2 == 0))
  {
    errno = EINVAL;
    return (DBL_MIN);
  }

  /*
   * code dans le cas ou les increments sont differents entre eux ou les
   * increments sont egaux mais differents de 1
   */
  if (inc1 != 1 && inc2 != 1)
  {
    double	*pc1, *pc2;
    double	*pend1, *pend2;

    if (inc1 > 1 && inc2 > 1)
    {
      for (pc1 = buf1, pend1 = buf1 + count, pc2 = buf2, pend2 = buf2 + count;
	   pc1 < pend1 && pc2 < pend2  ; pc1 += inc1, pc2 += inc2)
	dot += fabs (*pc1) * fabs (*pc2);

      return (dot);
    }

    if (inc1 >1 && inc2 < 0)
    {
      for (pc1 = buf1, pend1 = buf1 + count, pc2 = buf2 + count - 1,
	     pend2 = buf2; pc1 < pend1 && pc2 >= pend2; pc1 += inc1, pc2 += inc2)
	dot += fabs (*pc1) * fabs (*pc2);

      return (dot);
    }

    if (inc1 < 0 && inc2 > 1)
    {
      for (pc1 = buf1 + count -1, pend1 = buf1, pc2 = buf2,
	     pend2 = buf2 + count; pc1 >= pend1 && pc2 < pend2;
	   pc1 += inc1, pc2 += inc2)
	dot += fabs (*pc1) * fabs (*pc2);

      return (dot);
    }

    if (inc1 < 0 && inc2 < 0)
    {
      for (pc1 = buf1 + count - 1, pend1 = buf1, pc2 = buf2 + count - 1,
	     pend2 = buf2; pc1 >= pend1 && pc2 >= pend2; pc1 += inc1, pc2 += inc2)
	dot += fabs (*pc1) * fabs (*pc2);

      return (dot);
    }

  }

  /* code dans le cas ou un deux increments est egal a 1 et pas l'autre	*/

  /* code dans le cas ou les deux increments sont egaux a 1	*/
  m = count % 5;

  if (m != 0)
  {
    for (i = 0; i < m; i++)
      dot += buf1[i] * buf2[i];

    if ( count < 5 )
      return (dot);
  }

  for (i = m; i < (int) count; i += 5)
    dot += buf1[i] * buf2[i] + buf1[i + 1] * buf2[i + 1] + buf1[i + 2] * buf2[i + 2]
      + buf1[i + 3] * buf2[i + 3] + buf1[i + 4] * buf2[i + 4];

  return (dot);
}

/*
 * PROCEDURE    : dotinc_double
 *
 * INPUT       :
 * a            Tableau de reels double precision operande droit.
 * b            Tableau de reels double precision operande droit.
 * n            Nombre d'elements dans les tableaux "a" et "b".
 * inca         Increment du tableau "a".
 * incb         Increment du tableau "b".
 *
 * DESCRIPTION  :
 * La procedure calcule le produit scalaire des tableaux "a" et "b" sur "n"
 * reels double precision : S a[i] x b[i]. La procedure balaye le tableau "a"
 * par increment de "inca" et le tableau "b" par increment "incb".
 *
 * HISTORIQUE   :
 * 1.00 - 25/07/95 - Original.
 * 1.01 - 12/10/95 - Modification : passage en double
 */
double   dotinc_double (const double *a, const double *b, size_t n,
                        int inca, int incb)
{
  double		sum = 0.0;      /* produit scalaire     */
  register size_t i   = 0;        /* compteur             */

#define LSIZE   4
  if (n >= LSIZE) {       /* boucle deroulee LSIZE fois   */
    n -= LSIZE - 1;
    for (; i < n; i += LSIZE) {
      sum += *a * *b; a += inca; b += incb;
      sum += *a * *b; a += inca; b += incb;
      sum += *a * *b; a += inca; b += incb;
      sum += *a * *b; a += inca; b += incb;
    }
    n += LSIZE - 1;
  }
#undef  LSIZE
  for (; i < n; i++) {
    sum += *a * *b; a += inca; b += incb;
  }
  return (sum);
}

/*
 * PROCEDURE    : dot_mtx_rank
 *
 * INPUT       :
 * a            Tableau bidimensionnel de reels double precision.
 * rownb	Nombre de lignes du tableau "a".
 * colnb	Nombre de colonnes du tableau "a".
 *
 * OUTPUT	:
 * dotrow	Tableau contenant le resultat du produit scalaire ligne a ligne
 * dotcol	Tableau contenant le resultat du produit scalaire colonne a colonne
 *
 * DESCRIPTION  :
 * La procedure calcule le produit scalaire des rangees (lignes ou colonnes)
 * du tableau "a" de taille "rownb" x "colnb". Le resultat du produit scalaire
 * de chaque ligne par une autre (respectivement de chaque colonne par une
 * autre colonne), est place dans le tableau dotrow (respectivement dotcol).
 *
 * HISTORIQUE   :
 * 1.00 - 12/10/95 - Original.
 */
void	dot_mtx_rank (const double *a, size_t rownb, size_t colnb,
		      double *dotrow, double *dotcol)
{
  register size_t	i, j;

  /* produit scalaire sur les lignes */
  for (i = 0; i < rownb; i++)
    for (j = i; j < rownb; j++)
      *dotrow++ = dotinc_double (MIJ(a, i, 0, colnb),
				 MIJ(a, j, 0, colnb), colnb, 1, 1);

  /* produit sclaire sur les colonnes */
  for (i = 0; i < colnb; i++)
    for (j = i; j < colnb; j++)
      *dotcol++ = dotinc_double (MIJ(a, 0, i, colnb), MIJ(a, 0, j, colnb),
				 rownb, (int) colnb, (int) colnb);
}

/*
 * PROCEDURE    : dscal
 *
 * GLOBAL	:
 * errno	Code d'erreur.
 *
 * INPUT	:
 * buf1		Pointeur sur un tableau de doubles.
 * count	Taille des tableaux "buf1" et "buf2".
 * inc1		Increment dans le tableau "buf1".
 *
 * DESCRIPTION  :
 * La procedure multiplie les elements d'un tableau de taille "count" par une
 * constante "cst". Elle permet le calcul en utilisant des pas de "inc1" dans
 * le tableau "buf1".
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
int dscal (double *buf1, size_t count, double cst, int inc1)
{
  int	i;		/* compteur de boucle		*/
  int	m;		/* reste de la division entiere	*/

  if ((count == 0))
  {
    return (0);
  }
  /* code dans le cas ou l'increment est different de 1	*/
  if (inc1 != 1)
  {
    register int	i1;

    if (inc1 > 1)
    {
      for (i1 = 0; i1 < (int) count ; i1 += inc1)
	buf1[i1] = cst * buf1[i1];

      return (0);
    }

    if (inc1 < 0)
    {
      for (i1 = count - 1; i1 >= 0; i1 += inc1)
	buf1[i1] = cst * buf1[i1];

      return (0);
    }
  }

  /* code pour increment = 1	*/
  m = count % 5;

  if (m != 0)
  {
    for (i = 0; i < m; i++)
      buf1[i] = cst * buf1[i];

    if ( count < 5 )
      return (0);
  }

  for (i = m; i < (int) count; i += 5)
  {
    buf1[i] = cst * buf1[i];
    buf1[i + 1] = cst * buf1[i + 1];
    buf1[i + 2] = cst * buf1[i + 2];
    buf1[i + 3] = cst * buf1[i + 3];
    buf1[i + 4] = cst * buf1[i + 4];

  }
  return (0);
}

/*
 * PROCEDURE    : dswap
 *
 * GLOBAL	:
 * errno	Code d'erreur.
 *
 * INPUT	:
 * buf1		Pointeur sur un tableau de doubles.
 * buf2		Pointeur sur un tableau de doubles.
 * count	Taille des tableaux "buf1" et "buf2".
 * inc1		Increment dans le tableau "buf1".
 * inc2         Increment dans le tableau "buf1".
 *
 * DESCRIPTION  :
 * La procedure interchange les elements d'un tableau "buf1" de taille "count"
 * par les les elements d'un tableau "buf1" de taille "count". Elle permet
 * le changement en utilisant un pas "inc1" dans le tableau "buf1" et un pas inc2
 * dans le tableau "buf2".
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
int dswap (double *buf1, double *buf2, size_t count, int inc1, int inc2)
{
  int	i;	/* compteur de boucle		*/
  int	m;	/* reste de la division entiere	*/
  double	dtmp;	/* variable temporaire		*/

  if ((count == 0) || (inc1 == 0) || (inc2 == 0))
  {
    errno = EINVAL;
    return (-1);
  }

  /*
   * code dans le cas ou les increments sont differents entre eux ou les
   * increments sont egaux mais differents de 1
   */
  if (inc1 != 1 && inc2 != 1)
  {
    int	i1, i2;		/* increments dans les deux tableaux	*/

    if (inc1 > 1 && inc2 > 1)
    {
      for (i1 = 0, i2 = 0; i1 < (int) count && i2 < (int) count; i1 += inc1,
	     i2 += inc2)
      {
	dtmp = buf1[i1];
	buf1[i1] = buf2[i2];
	buf2[i2] = dtmp;
      }
      return (0);
    }

    if (inc1 > 1 && inc2 < 0)
    {
      for (i1 = 0, i2 = (int) count - 1; i1 < (int) count && i2 >= 0; i1 += inc1,
	     i2 += inc2)
      {
	dtmp = buf1[i1];
	buf1[i1] = buf2[i2];
	buf2[i2] = dtmp;
      }
      return (0);
    }

    if (inc1 < 0 && inc2 > 1)
    {
      for (i1 = (int) count - 1, i2 = 0; i1 >= 0 && i2 < (int) count; i1 += inc1,
	     i2 += inc2)
      {
	dtmp = buf1[i1];
	buf1[i1] = buf2[i2];
	buf2[i2] = dtmp;
      }
      return (0);
    }

    if (inc1 < 0 && inc2 < 0)
    {
      for (i1 = count - 1, i2 = count - 1; i1 >= 0 && i2 >= 0; i1 += inc1,
	     i2 += inc2)
      {
	dtmp = buf1[i1];
	buf1[i1] = buf2[i2];
	buf2[i2] = dtmp;
      }
      return (0);
    }
  }

  /* code pour inc1 et inc2 egaux a un	*/
  m = count % 3;

  if (m != 0)
  {
    for (i = 0; i < m; i++)
    {
      dtmp = buf1[i];
      buf1[i] = buf2[i];
      buf2[i] = dtmp;
    }

    if ( count < 3)
      return (0);
  }

  for (i = m; i < (int) count; i += 3)
  {
    dtmp = buf1[i];
    buf1[i] = buf2[i];
    buf2[i] = dtmp;

    dtmp = buf1[i + 1];
    buf1[i + 1] = buf2[i + 1];
    buf2[i + 1] = dtmp;

    dtmp = buf1[i + 2];
    buf1[i + 2] = buf2[i + 2];
    buf2[i + 2] = dtmp;

  }
  return (0);

}

/*
 * PROCEDURE    : idamax
 *
 * GLOBAL	:
 * errno	Code d'erreur.
 *
 * INPUT	:
 * buf1		Pointeur sur un tableau de doubles.
 * count	Taille des tableaux "buf1" et "buf2".
 * inc1		Increment dans le tableau "buf1".
 *
 * DESCRIPTION  :
 * La procedure recherche l'indice de l'elements maximum en valeur absolue
 * d'un tableau de reel double precision "buf1" de taille "count". Elle
 * permet la recherche en utilisant un pas "inc1" dans le tableau "buf1".
 *
 * RETOUR	:
 * En cas de succes, l'indice de l'element maximum est retourne.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 11/03/78 - Original. Jack Dongarra
 * 1.01 - 21/08/90 - Correction du probleme d'increment negatif.
 * 1.02 - 29/05/95 - Amelioration et correction de bugs.
 */
int idamax (double *buf, size_t count, int inc)
{
  int	i;		/* compteur de boucle		*/
  int	imax;

  if ((count == 0) || (inc == 0))
  {
    errno = EINVAL;
    return (-1);
  }
  /* code dans le cas ou l'increment est different de 1	*/
  if (inc != 1)
  {
    register int	i1;

    if (inc > 1)
    {
      for (i1 = 0, imax = 0; i1 < (int) count ; i1 += inc)
	imax = (fabs(buf[i1]) > fabs(buf[imax])) ? i1 : imax;

      return (imax);
    }

    if (inc < 0)
    {
      for (i1 = count - 1, imax = count - 1; i1 >= 0; i1 += inc)
	imax = (fabs(buf[i1]) > fabs(buf[imax])) ? i1 : imax;

      return (imax);
    }
  }

  /* code pour increment = 1	*/
  for (i = 0, imax = 0; i < (int) count; i++)
    imax = (fabs(buf[i]) > fabs(buf[imax])) ? i : imax;
  return (imax);
}

/*
 * PROCEDURE    : show_mtx
 *
 *
 * INPUT	:
 * buf		Pointeur sur une matrice de doubles.
 * raw		Nombre de ligne de la matrice "buf".
 * col		Nombre de ligne de la matrice "buf".
 * maxcol	Nombre d'elements sur une ligne de la matrice "buf".
 *
 * DESCRIPTION  :
 * La procedure affiche la matrice "buf de taille "raw" x "col".
 *
 *
 * HISTORIQUE   :
 * 1.00 - 30/05/95 - Original.
 */
void	show_mtx (double *buf, size_t raw, size_t col)
{
  register size_t	i, j;

  for (i = 0; i < raw; i++)
    for (j = 0; j < col; j++)
      printf ("%.11f%c", buf[i *col + j], (j == col - 1) ? '\n' : '\t');
}


#ifdef	OUTIL2_MAIN

#include	<stdlib.h>
#include	<stdio.h>
#include	<memory.h>

#include	"mtx_tool.h"

#define	ROW	7
#define COL	3

void	main (void)
{
  int	i;		/* compteur de boucle */
#if 0
  double a[ROW][COL] = {1.0, 2.0, 3.0, -5.0, -6.0, 7.0, 10.0,
			5.0, 6.0, 7.0, 8.0, 9.0, 15.0, 9.0,
			9.0, 10.0, 11.0, -9.0, -5.0, 41.0, 3.0};
  double a[ROW][COL] = {1.0, 2.0, 3.0, -5.0,
			5.0, 6.0, 7.0, 8.0,
			9.0, 10.0, 11.0, -9.0,
			10.0, 9.0, 8.0, 7.0};
#endif
  double a[ROW][COL] = {1.0, 2.0, 3.0,
			5.0, 6.0, 7.0,
			9.0, 10.0, 11.0,
			10.0, 9.0, 8.0,
			7.0, -5.0, 9.0,
			4.0,5.0, 1.0,
			-1.0, 8.0, 0.0};

  double	dotrow[ROW * ROW];
  double	dotcol[ROW * ROW];
  double	ps;

  memset (dotrow, 0, ROW * ROW * sizeof (double));
  memset (dotcol, 0, ROW * ROW * sizeof (double));
  show_mtx (&a[0][0], ROW, COL);
  printf("\n\n");
  ps = dotinc_double (&a[0][0], &a[0][0], COL, 1, 1);
  printf("ps = %lf\n",ps);
  dot_mtx_rank (&a[0][0], ROW, COL, dotrow, dotcol);

  for (i = 0; i < ROW * ROW; i++)
    printf ("dotrow[%d] = %.6e\tdotcol[%d] = %.6e\n", i, dotrow[i], i, dotcol[i]);
}
#endif	/* OUTIL2_MAIN	*/
