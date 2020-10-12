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

  DESCRIPTION	: Le fichier contient la gestion de la memoire des
                  pyramides utiles a l'estimation.

*/

/* Inclusion des fichiers generaux                  */
#include "type.h"
#include "macro.h"
#include "constant.h"

/* Inclusion des prototypes des procedures internes */
#include "mem_est.h"

/* Inclusion des prototypes des procedures locales  */
#include "../memoire/memoire.h"


/*
 * PROCEDURE	: init_mem_est
 *
 * INPUTS      :
 * nblig          Nombre de lignes du niveau de base de la pyramide.
 * nbcol          Nombre de colonnes du niveau de base de la pyramide.
 * niv_final      Dernier niveau de la pyramide.
 *
 * INPUT/OUTPUT      :
 * init_mem_estIsDone	  Si 1, indique que la memoire pour l'estimation
 *		  a ete initialisee.
 *
 * OUTPUT       :
 * pyr_fl1	  Gradients deplaces suivant x.
 * pyr_fl2	  Gradients deplaces suivant y.
 * pyr_fl3	  DFD: I(pi+depl,t+1)-I(pi,t).
  pyr_support	  estimation support pyramid.
 *
 * DESCRIPTION	:
 * La procedure alloue de la memoire pour les pyramides "pyr_fl1", "pyr_fl2",
 * "pyr_fl3" et "pyr_support" utiles a l'estimation du modele de mouvement.
 *   - "pyr_fl1", "pyr_fl2" et "pyr_fl3" sont des pyramides de flottants
 *     a "niv_final+1" niveaux. Ces pyramides contiennent respectivement:
 *     le gradient deplace suivant x, le gradient deplace suivant y et la
 *     DFD c'est-a-dire I(pi+depl,t+1)-I(pi,t)
 *   - "pyr_support" est une pyramide de short a "niv_final+1" niveaux.
 *     Cette pyramide contient les coefficients de ponderation.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool init_mem_est(int nblig, int nbcol, int &niv_final,
		  bool *init_mem_estIsDone,
		  TImageFloat *pyr_fl1, TImageFloat *pyr_fl2,
		  TImageFloat *pyr_fl3, TImageShort *pyr_support)
{
  int  i;

  if (*init_mem_estIsDone == false) {
    if (Mem_pyramide_float(pyr_fl1, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide_float(pyr_fl2, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide_float(pyr_fl3, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide(pyr_support, nblig, nbcol, niv_final) == false)
      return false;
    *init_mem_estIsDone = true;
    return true;
  }

  // La memoire est deja initialisee, mais il faut voir si la taille
  // est correcte
  if (pyr_fl1[0].nbli != nblig || pyr_fl1[0].nbco != nbcol) {
    for (i = 0; i <= niv_final; i++) {
      free((char *) pyr_fl1[i].ad);
      pyr_fl1[i].ad = NULL;

      free((char *) pyr_fl2[i].ad);
      pyr_fl2[i].ad = NULL;

      free((char *) pyr_fl3[i].ad);
      pyr_fl3[i].ad = NULL;

      free((char *) pyr_support[i].ad);
      pyr_support[i].ad = NULL;
    }
    if (Mem_pyramide_float(pyr_fl1, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide_float(pyr_fl2, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide_float(pyr_fl3, nblig, nbcol, niv_final) == false)
      return false;
    if (Mem_pyramide(pyr_support, nblig, nbcol, niv_final) == false)
      return false;
    *init_mem_estIsDone = true;
    return true;
  }

  return true;
}


/*
 * PROCEDURE	: efface_memoire_pyr_est
 *
 * INPUT       :
 * Num_niv_max	  numero du dernier niveau de la pyramide
 *
 * INPUT       :
 * pyr_fl1	  Gradients deplaces suivant x.
 * pyr_fl2	  Gradients deplaces suivant y.
 * pyr_fl3	  DFD: I(pi+depl,t+1)-I(pi,t).
  pyr_support	  estimation support pyramid.
 *
 * OUTPUT       : Aucune
 *
 * DESCRIPTION	:
 * La procedure efface la memoire allouee aux differentes pyramides
 * specifiques a l'estimation.
 */

void efface_memoire_pyr_est(int Num_niv_max, bool *init_mem_estIsDone,
			    TImageFloat *pyr_fl1, TImageFloat *pyr_fl2,
			    TImageFloat *pyr_fl3, TImageShort *pyr_support)
{
  if (*init_mem_estIsDone == true) {
    free_p_fl(pyr_fl1,Num_niv_max);  // gradient deplace suivant x
    free_p_fl(pyr_fl2,Num_niv_max);  // gradient deplace suivant y
    free_p_fl(pyr_fl3,Num_niv_max);  // DFD c-a-d I(pi+depl,t+1)-I(pi,t)
    free_p(pyr_support,Num_niv_max); // coefficients de ponderation
    *init_mem_estIsDone = false;
  }
}
