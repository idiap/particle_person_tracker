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

#include	"arithm.h"


/*
 * PROCEDURE    : dsign
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * a		Nombre reel en double precision.
 * b		Nombre reel en double precision.
 *
 * DESCRIPTION  :
 * La procedure retourne la valeur de |a| affectee du signe de b. Soit
 *  |a| si b > 0
 * -|a| si b < 0
 *
 * RETOUR	:
 * En cas de succes, la valeur a affectee du signe de b est retournee.
 * Sinon, la valeur DBL_MIN est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 28/05/95 - Original.
 */
double dsign (double a, double b)
{
	double	ret;

	if (b == 0.0)
	{
		errno = EDOM;
		return (DBL_MIN);
	}
	ret = fabs (a) * SIGN(b);
	return (ret);
}

/*
 * PROCEDURE    : dtrans
 *
 * GLOBAL	:
 * errno	Code d'erreur
 *
 * INPUT	:
 * a		Pointeur sur le tableau.
 *
 * DESCRIPTION  :
 * La procedure transpose une matrice carre de reels double precision "a".
 * Soit a[i][j] = a[j][i]
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 * HISTORIQUE   :
 * 1.00 - 28/05/95 - Original.
 */
int dtrans (double *a, size_t raw, size_t col)
{
	register int	 i, j;
	register double	dtmp;

	if (col == 0 || raw == 0)
	{
		errno = EINVAL;
		return (-1);
	}

	for (i = 0; i < (int) raw; i++)
		for (j = 0; j < (int) col; j++)
			SWAP(a[i * (int) col + j], a[j * (int) col + i], dtmp);
	return (0);
}


#ifdef	UTIL_MAIN

#include	<conio.h>
#include	<ctype.h>
#include	<direct.h>
#include	<limits.h>
#include	<stdio.h>
#include	<stdlib.h>

#define	RAW	5
#define	COL	4

#include	<csl1.h>
#include	<csl4.h>
#include	<csl8.h>

void	main (int argc, char *argv[])
{
	double	a,b, res, tmp(0);

	double	arr[RAW][COL] =     {{1.0, 2.0,  6.0, 15.0},
				      {8.0, 0.0,  1.0, -3.0},
				      {9.0, 4.0,  3.0,  9.0},
				      {1.0, 5.0, -5.0, 10.0},
				      {-3.0, 1.0, -1.0, -1.0}};

	static double	m[24]=	{11.0,12.0,13.0,14.0,15.0,16.0,
                        	 21.0,22.0,23.0,24.0,25.0,26.0,
                        	 31.0,32.0,33.0,34.0,35.0,36.0,
                        	 41.0,42.0,43.0,44.0,45.0,46.0,
                        	};

	int	i,j;

	if (argc != 3)	/* affichage du chemin d'acces	*/
	{
		printf ("%s nombre 1 nombre 2\n", argv[0]);
		exit (1);
	}

	printf ("%s\t %s\n", argv[1], argv[2]);
	a = atol (argv[1]);
	b = atol (argv[2]);

	res = dsign (a, b);
	if (errno == EZERO)
	{
		perror (" b = 0\n");
		exit (1);
	}
	printf("a = %lf\t b = %lf\tres = %lf\n", a, b, res);

	printf ("test de swap\n");

	printf ("avant\n");
	printf ("n1 = %lf\t n2 = %lf\n", a, b);

	SWAP(a,b,tmp);

	printf ("apres\n");
	printf ("n1 = %lf\t n2 = %lf\n", a, b);

#if 0

	printf ("matrice dans proc avant trn\n");
	for (i = 0; i < RAW; i++)
		for (j = 0; j < COL; j++)
			printf ("%5.1lf%c", arr[i * COL + j], (j == COL - 1) ? '\n' : ' ');

	dtrans (arr, RAW, COL);

	printf ("matrice dans proc apres trn\n");
	for (i = 0; i < COL; i++)
		for (j = 0; j < RAW; j++)
			printf ("%5.1lf%c", arr[i * RAW + j], (j == RAW - 1) ? '\n' : ' ');
#endif

   puts("\nMatrix before transpose:\n"); prtmtx(" %2.0f",&arr[0][0],RAW,COL);
   mtrp(&arr[0][0],RAW,COL); puts("\nMatrix after the transpose\n");
   prtmtx(" %2.0f",&arr[0][0],COL,RAW);
}

#endif	/* UTIL_MAIN	*/
