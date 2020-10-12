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

  DESCRIPTION	: Le fichier contient la procedure permettant l'inversion
                  de matrices en exploitant leur symetrie.

*/


/* inclusion des fichiers standards.                */
#include <math.h>


/* Inclusion des prototypes des procedures internes */
#include "inverse_mat_sym.h"

/* Inclusion des prototypes des procedures locale.  */
#include "invert.h"

/* Definition des macro-instructions.               */
#define TOL            1.0e-5
#define ERREUR_TOLEREE 1.0e-4


#ifndef FALSE
#  define	FALSE	0
#endif

#ifndef TRUE
#  define	TRUE	1
#endif


/*
 * PROCEDURE	: inverse_mat_sym
 *
 * INPUT       :
 * A              Matrice a inverser, de dimension m x m.
 *
 * OUTPUT       :
 * invA           Matrice inversee, de dimension m x m.
 *
 * INPUT       :
 * m              Taille de la matrice a inverser.
 *
 * DESCRIPTION	:
 * La procedure inverse une matrice symetrique de dimension m x m.
 * La procedure renvoie TRUE si l'inversion s'est bien passee, FALSE dans
 * le cas contraire.
 * Remarque : Comme le Fortran, les fonctions appelees travaillent sur des
 *            matrices dont le 1er element est [1] et non pas [0] comme en C.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

int inverse_mat_sym(double *A, double *invA, int m)
{
  double	erreur, sum, tmp;
  int		*ipvt;
  double	rcond, *z;
  int		job = 01;	/* recherche de l'inverse seulement */
  int		i, j, k;
  double	det[2];


  ipvt = (int *) malloc(m * sizeof(int));
  z    = (double *) malloc(m * sizeof(double));

  /* copie de A[][] dans invA[][] */
  for (i = 0; i < m*m; i++)
    invA[i] = A[i];

  /* calcul de A-1 */
  dgeco (invA, (size_t) m, (size_t) m, ipvt, &rcond, z);

  dgedi (invA, (size_t) m, (size_t)m, ipvt, det, z, job);

  /* calcul de l'erreur commise lors de l'inversion de A[][] */
  /* erreur = (I-A[][]*invA[][])2                            */
  erreur = 0.0;
  for (i = 0; i < m; i++)
    {
      for (j = 0; j < m; j++)
	{
	  for (sum = 0.0, k = 0; k < m; k++)
	    {
	      sum += *(A + i * m + k) * *(invA + k * m + j);
	    }
	  if (i == j)
	    erreur += (tmp=(1.0 - sum), tmp * tmp);
	  else
	    erreur += (sum * sum);
	}
    }

  free (ipvt);
  free (z);

  /* code retour */
  if(erreur < ERREUR_TOLEREE)
    return (TRUE);
  else
    return (FALSE);
}

#undef TOL
#undef ERREUR_TOLEREE



#ifdef	INVERSE_MAT_SYM_MAIN

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>


#define	RAW	8
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
  double	matrix[RAW][COL];
  double	mat_inv[RAW][COL];
  int		i, j, k;

  matrix[0][0] =  350514.31775;
  matrix[0][1] = -43392.26968;
  matrix[0][2] = -633009.94911;
  matrix[0][3] = -3206622.86443;
  matrix[0][4] =  2755060.74883;
  matrix[0][5] = 1402140.54852;
  matrix[0][6] = 196323194.73981;
  matrix[0][7] = -51653458.37379;

  matrix[1][1] = 251753.09966;
  matrix[1][2] = 2755060.74883;
  matrix[1][3] = 1402140.54852;
  matrix[1][4] = -583941.20948;
  matrix[1][5] = -2290553.82263;
  matrix[1][6] = -7261055.17965;
  matrix[1][7] = 225678987.48700;

  matrix[2][2] = 280988727.20367;
  matrix[2][3] = 19570272.77812;
  matrix[2][4] = -54367818.56046;
  matrix[2][5] = -84665527.46386;
  matrix[2][6] = 1324504585.19172;
  matrix[2][7] = 152449130.10465;

  matrix[3][3] = 403567458.14788;
  matrix[3][4] = -84665527.46386;
  matrix[3][5] = -71223731.15191;
  matrix[3][6] = 152449130.10465;
  matrix[3][7] = -489796136.58222;

  matrix[4][4] = 222048044.11205;
  matrix[4][5] = 47106763.38080;
  matrix[4][6] = 63086860.82195;
  matrix[4][7] = -1529574663.16409;

  matrix[5][5] = 310344519.95086;
  matrix[5][6] = -1529574663.16409;
  matrix[5][7] = 1662131716.67677;

  matrix[6][6] = 716624167183.69299;
  matrix[6][7] = -230372138615.66486;

  matrix[7][7] = 835720352485.43042;


  /* complete le triangle inferieur de la matrice symetrique	*/
  for (i = 0; i < COL; i++)
    for (j = i + 1; j < RAW; j++)
      matrix[j][i] = matrix[i][j];

  printf ("Matrice a inverser :\n");
  show_mtx (&matrix[0][0], RAW, COL);

  if (inverse_mat_sym(&matrix[0][0], &mat_inv[0][0], COL) )
    {
      printf ("\nMatrice inverse :\n");
      show_mtx (&mat_inv[0][0], RAW, COL);
    }
  else
    printf("ERREUR dans l'inversion de la matrice......\n");
}


#undef COL
#undef RAW

#endif

















