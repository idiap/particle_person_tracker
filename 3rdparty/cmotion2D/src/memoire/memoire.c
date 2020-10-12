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
  
 DESCRIPTION	: Le fichier contient les procedures d'allocation et de
                  liberation de la memoire des pyramides.

*/

/* Inclusion des fichiers standards                 */
#include <stdio.h>

/* Inclusion des fichiers generaux                  */
#include "type.h"
#include "macro.h"

/* Inclusion des prototypes des procedures internes */
#include "memoire.h"

#define LEVEL_ROW_MIN_SIZE 2
#define LEVEL_COL_MIN_SIZE 2

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

/*
 * PROCEDURE	: Mem_pyramide
 *
 * OUTPUT	:
 * imag           Pointeur sur la pyramide de short.
 *
 * INPUTS      :
 * nbli           Nombre de lignes de la zone memoire a allouer.
 * nbco           Nombre de colonnes de la zone memoire a allouer.
 * nboct          Nombre d'octets par pixels.
 * niv_max        Niveau maximal de la pyramide.
 *
 *
 * DESCRIPTION	:
 * La procedure alloue la memoire necessaire a la pyramide "imag" de short.
 * L'allocation est faite a partir du niveau de base de taille nbli*nbco,
 * jusqu'au niveau maximal "niv_max". Chaque niveau occupe un espace 4 fois
 * plus reduit que celui de son pere.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Mem_pyramide(TImageShort *imag, int nbli, int nbco, int & niv_max)
{
  int	i;
  if (DEBUG_LEVEL1) printf("Debut Mem_pyramide()\n");

  if (DEBUG_LEVEL2) printf("nbli=%d nbco=%d niv 0\n", nbli, nbco);

  
  if (nbli < LEVEL_ROW_MIN_SIZE) {
    // Aucun niveau ne peut etre alloue
    niv_max = -1;
    return false;
  }
  if (nbco < LEVEL_COL_MIN_SIZE) {
    // Aucun niveau ne peut etre alloue
    niv_max = -1;
    return false;
  }

  imag[0].nbli  = nbli;
  imag[0].nbco  = nbco;
  if (niv_max < 0) {
    fprintf(stderr,
	    "\nBad hight level in the pyramids : must be equal to zero or positive.\n");
    return false;
  }
  if ( (imag[0].ad = (short *) calloc((unsigned) (nbli * nbco),
				     sizeof(short))) == NULL) {
    fprintf(stderr,"\nCan not allocate memory for the pyramids.\n");
    return false;
  }
  for (i=1;i<=niv_max;i++) {
    nbli = nbli/2;
    nbco = nbco/2;

    if (nbli < LEVEL_ROW_MIN_SIZE) {
      // Les images sont trop petites, on arrete l'allocation et on met a jour
      // le dernier niveau exploitable
      niv_max = i - 1;
      break;
    }
    if (nbco < LEVEL_COL_MIN_SIZE) {
      // Les images sont trop petites, on arrete l'allocation et on met a jour
      // le dernier niveau exploitable
      niv_max = i - 1;
      break;
    }

    if (DEBUG_LEVEL2) printf("nbli=%d nbco=%d niv %i\n", nbli, nbco, i);

    imag[i].nbli = nbli;
    imag[i].nbco = nbco;
    if( (imag[i].ad = (short *)calloc((unsigned)(nbli*nbco),
				      sizeof(short))) == NULL) {
      fprintf(stderr,"\nCan not allocate memory for the pyramids.\n");
      return false;
    }
  }
  if (DEBUG_LEVEL1) printf("Fin Mem_pyramide()\n");
  return true;
}


/*
 * PROCEDURE	: Mem_pyramide_float
 *
 * OUTPUT	:
 * imag           Pointeur sur la pyramide de flottants.
 *
 * INPUTS      :
 * nbli           Nombre de lignes de la zone memoire a allouer.
 * nbco           Nombre de colonnes de la zone memoire a allouer.
 * niv_max        Niveau maximal de la pyramide.
 *
 *
 * DESCRIPTION	:
 * La procedure alloue la memoire necessaire a la pyramide de flottants "imag".
 * L'allocation est faite a partir du niveau de base de taille nbli*nbco,
 * jusqu'au niveau maximal "niv_max". Chaque niveau occupe un espace 4 fois
 * plus reduit que celui de son pere.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Mem_pyramide_float(TImageFloat *imag, int nbli, int nbco, int & niv_max)
{
  int	i;

  if (DEBUG_LEVEL1) printf("Debut Mem_pyramide_float()\n");
  if(niv_max < 0) {
    fprintf(stderr,
	    "\nBad hight level in the pyramids : must be equal to zero or positive.\n");
    return false;
  }

  if (DEBUG_LEVEL2) printf("nbli=%d nbco=%d niv 0\n", nbli, nbco);
    
  if (nbli < LEVEL_ROW_MIN_SIZE) {
    // Aucun niveau ne peut etre alloue
    niv_max = -1;
    return false;
  }
  if (nbco < LEVEL_COL_MIN_SIZE) {
    // Aucun niveau ne peut etre alloue
    niv_max = -1;
    return false;
  }


  imag[0].nbli = nbli;
  imag[0].nbco = nbco;

  if( (imag[0].ad = (float *)calloc((unsigned)(imag[0].nbli*imag[0].nbco),
				    sizeof(float)))==NULL) {
    fprintf(stderr,"\nCan not allocate memory for the pyramids.\n");
    return false;
  }
  for(i=1;i<=niv_max;i++) {
    imag[i].nbli = imag[i-1].nbli/2;
    imag[i].nbco = imag[i-1].nbco/2;

    if (imag[i].nbli < LEVEL_ROW_MIN_SIZE) {
      // Les images sont trop petites, on arrete l'allocation et on met a jour
      // le dernier niveau exploitable
      niv_max = i - 1;
      break;
    }
    if (imag[i].nbco < LEVEL_COL_MIN_SIZE) {
      // Les images sont trop petites, on arrete l'allocation et on met a jour
      // le dernier niveau exploitable
      niv_max = i - 1;
      break;
    }
    if (DEBUG_LEVEL2) printf("nbli=%d nbco=%d niv %i\n", imag[i].nbli, imag[i].nbco, i);

    if( (imag[i].ad = (float *)calloc((unsigned)(imag[i].nbli*imag[i].nbco),
				      sizeof(float)))==NULL) {
      fprintf(stderr,"\nCan not allocate memory for the pyramids.\n");
      return false;
    }
  }
  if (DEBUG_LEVEL1) printf("Fin Mem_pyramide_float()\n");
  return true;
}



/*
 * PROCEDURE	: free_p
 *
 * INPUTS      :
 * im             Pointeur sur la pyramide de short.
 * nb_niv         Nombre de niveaux de la pyramide.
 *
 * DESCRIPTION	:
 * La procedure libere la memoire allouee a une pyramide de short.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void free_p(TImageShort im[], int nb_niv)
{
  int	i;

  for(i=0;i<nb_niv;i++) {
    free((char *)im[i].ad);
    im[i].ad = NULL;
  }
}



/*
 * PROCEDURE	: free_p_fl
 *
 * INPUTS      :
 * im             Pointeur sur la pyramide de flottants.
 * nb_niv         Nombre de niveaux de la pyramide.
 *
 * DESCRIPTION	:
 * La procedure libere la memoire allouee a une pyramide de flottants.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void free_p_fl(TImageFloat im[], int nb_niv)
{
  int	i;
  for(i=0;i<nb_niv;i++)  {
    free((char *)im[i].ad);
    im[i].ad = NULL;
  }
}

#undef LEVEL_ROW_MIN_SIZE
#undef LEVEL_COL_MIN_SIZE
