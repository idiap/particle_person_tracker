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

  DESCRIPTION	: Le fichier contient la gestion de la resolution du
                  systeme d'equations lineaires : A*X = B, avec A, X et B
                  des matrices. La methode utilisee consiste a exploiter
		  la symetrie de la matrice A.

*/

/* inclusion des fichiers standards.                */
#include <math.h>

/* Inclusion des fichiers generaux                  */
#include "constant.h"

/* Inclusion des prototypes des procedures internes */
#include "resoud_mat_sym.h"

/* Inclusion des prototypes des procedures locale.  */
#include "invert.h"

/* Definition des macro-instructions.               */
#define TOL            1.0e-5
#define ERREUR_TOLEREE 1.0e-4



/*
 * PROCEDURE	: resoud_mat_sym
 *
 * INPUTS      :
 * A              Matrice de dimension "m" x "n".
 * B              Matrice de dimension "m".
 *
 * OUTPUT       :
 * X              Matrice de dimension "n", solution du systeme A*X = B.
 *
 * INPUT       :
 * m              Nombre de colonnes des matrices.
 * n              Nombre de lignes des matrices.
 *
 * DESCRIPTION	:
 * La procedure resoud le systeme d'equations lineaires A*X = B en
 * exploitant la symetrie de la matrice A.
 * La procedure renvoie TRUE si la resolution s'est bien passee, FALSE dans
 * le cas contraire.
 *
 * HISTORIQUE   :
 * 1.00 - 14/11/95 - Original.
 */

int resoud_mat_sym(double *A, double *B, double *X, int m, int n)
{
  double	erreur, sum, tmp;
  int		*ipvt;
  double	rcond, *z;
  double	*invA;
  int		job = 01;	/* recherche de l'inverse seulement */
  int		i, j;
  double	det[2];

  ipvt = (int *) malloc(m * sizeof(int));
  z    = (double *) malloc(m * sizeof(double));
  invA = (double *) malloc(m * n * sizeof(double));

  /* copie de A[][] dans invA[][] */
  for (i = 0; i < m*m; i++)
    invA[i] = A[i];

  for (i = 0; i < n; i++)
    X[i] = 0.0;

  /* calcul de A-1 */
  dgeco (invA, (size_t) n, (size_t) m, ipvt, &rcond, z);
  dgedi (invA, (size_t) n, (size_t) m, ipvt, det, z, job);

  /* resoud le systeme a[][] . x[] = b[] en calculant x[] = a-1[][] . b[] */
  for ( i = 0; i < n; i++)
    for ( j = 0; j < m; j++)
      {
	X[i] += *(invA + i * m + j) * B[j];
      }

  /* calcul de l'erreur commise lors de la resolution de A[][]*X[] = B[] */
  /* erreur = (B[] - A[][]*X[])2                                         */
  erreur = 0.0;
  for (i = 0; i < n;i++)
    {
      for (sum = 0.0, j = 0; j < m; j++)
	sum += *(A + i * m + j) * X[j];
      erreur += (tmp = (B[i] - sum), tmp * tmp);
    }

  free (ipvt);
  free (z);
  free (invA);

  /* code retour */
  if(erreur < ERREUR_TOLEREE)
    return (TRUE);
  else
    return (FALSE);

}


#undef TOL
#undef ERREUR_TOLEREE


#ifdef	RESOUD_MAT_SYM_MAIN

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>


#define	RAW	8
#define	COL	RAW


/*
 * PROCEDURE    : main
 *
 * DESCRIPTION  :
 * La procedure teste la procedure resoud_mat_sym.
 *
 * HISTORIQUE   :
 * 1.00 - 01/06/95 - Original.
 */
void main (void)
{

  int i, j;
  double A[RAW][COL], B[RAW], X[COL];

  B[0] = 837.90442;
  B[1] = -277.92770;
  B[2] = -19864.66760;
  B[3] = -27643.76329;
  B[4] = 10998.16025;
  B[5] = 12885.79131;
  B[6] = 9123575.11870;
  B[7] = -13962428.44980;

  A[0][0] =  350514.31775;
  A[0][1] = -43392.26968;
  A[0][2] = -633009.94911;
  A[0][3] = -3206622.86443;
  A[0][4] =  2755060.74883;
  A[0][5] = 1402140.54852;
  A[0][6] = 196323194.73981;
  A[0][7] = -51653458.37379;

  A[1][1] = 251753.09966;
  A[1][2] = 2755060.74883;
  A[1][3] = 1402140.54852;
  A[1][4] = -583941.20948;
  A[1][5] = -2290553.82263;
  A[1][6] = -7261055.17965;
  A[1][7] = 225678987.48700;

  A[2][2] = 280988727.20367;
  A[2][3] = 19570272.77812;
  A[2][4] = -54367818.56046;
  A[2][5] = -84665527.46386;
  A[2][6] = 1324504585.19172;
  A[2][7] = 152449130.10465;

  A[3][3] = 403567458.14788;
  A[3][4] = -84665527.46386;
  A[3][5] = -71223731.15191;
  A[3][6] = 152449130.10465;
  A[3][7] = -489796136.58222;

  A[4][4] = 222048044.11205;
  A[4][5] = 47106763.38080;
  A[4][6] = 63086860.82195;
  A[4][7] = -1529574663.16409;

  A[5][5] = 310344519.95086;
  A[5][6] = -1529574663.16409;
  A[5][7] = 1662131716.67677;

  A[6][6] = 716624167183.69299;
  A[6][7] = -230372138615.66486;

  A[7][7] = 835720352485.43042;


  /* complete le triangle inferieur de la matrice symetrique	*/
  for (i = 0; i < COL; i++)
    for (j = i + 1; j < RAW; j++)
      A[j][i] = A[i][j];

  printf ("Soit le systeme A[][] . X[] = B[] a resoudre avec :\n\n");
  printf ("matrice A[][] :\n");
  show_mtx (&A[0][0], RAW, COL);

  printf ("\nmatrice B[] :\n");
  /* show_mtx (B, 1, COL); */
  for (i = 0; i < COL; i++)
	printf("B[%d] = %3.15f \n", i, B[i]);

  if (resoud_mat_sym(&A[0][0], B, X, COL, RAW) )
    {
      printf("\nMatrice X[] resultat de A[][]. X[] = B[]\n");
      /* show_mtx (&X[0], 1, COL); */
      for (i = 0; i < COL; i++)
	printf("X[%d] = %3.15f \n", i, X[i]);
    }
  else
    printf("ERREUR dans la resolution du systeme......\n");
}

#undef COL
#undef RAW

#endif
