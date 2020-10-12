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

  DESCRIPTION	: Le fichier contient la gestion et la construction
                  des pyramides d'images et de gradients.

*/

/* Inclusion des fichiers generaux                  */
#include   "type.h"
#include   "macro.h"
#include   "constant.h"

/* Inclusion des prototypes des procedures internes */
#include   "pyramide.h"

/* Inclusion des prototypes des procedures locales  */
#include   "filt_gauss.h"
#include   "multigr.h"
#include   "gradient.h"

#include   "memoire.h"

/*
 * PROCEDURE	: Etablit_pyr
 *
 * OUTPUTS:
 * ima            Pointeur sur la pyramide image de short.
 * gx             Pointeur sur la pyramide du gradient en x.
 * gy             Pointeur sur la pyramide du gradient en y.
 *
 * INPUTS      :
 * nb_niv                  Nombre de niveaux de la pyramide.
 * sigma_gauss             Variance du filtre gaussien.
 * coef_filt_avant_ss      Coefficient du filtre utilise avant le
 *                         sous-echantionnage.
 * taille_filtre_gradient  Taille du filtre utilise pour le calcul des
 *                         gradients spatiaux. La taille est egale a 3, 7 ou 9.
 * mode                    Indique si l'on veut construire les pyramides de
 *                         gradients. Si mode = 1, les gradients sont calcules.
 * mem                     Indique si la memoire requise pour les pyramides
 *                         existe. Si mem = 0, la memoire existe. Si mem = 1,
 *                         il faut la reserver.
 * PRINTF_ON	  Si 1, mode debug par affichage de printf().
 *
 * DESCRIPTION	:
 * La procedure etablie une pyramide d'images et de gradients.
 * On suppose que ima[0] contient l'image de base non filtre. Chaque image de
 * base est d'abord filtree par un filtre gaussien de variance "sigma_gauss".
 * Puis, les differents niveaux de la pyramide d'images sont obtenus par
 * filtrage et sous-echantillonnage. De plus, si "mode = 1" les pyramides
 * de gradients sont construites.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool Etablit_pyr(TImageShort *ima, TImageFloat *gx, TImageFloat *gy,
		 int &nb_niv, double sigma_gauss, double coef_filt_avant_ss,
		 int taille_filtre_gradient, int mode, int mem)
{
  int    i;
  int niv_max = nb_niv-1;
  bool state;

  /* Etablissement de la pyramide de l'image */
  /* filtrage gaussien de l'image de base    */
  if (filtre_gauss(&ima[0],&ima[0],&sigma_gauss) == false)
    return false;
  /* construction de la pyramide de l'image  */
  /* par filtrage et sous-echantionnage      */

  // Warning: niv_max est potentiellement modifie
  if (Reduit_pyramide(ima,niv_max,coef_filt_avant_ss,mem) == false)
    return false;

  /* Etablissement des images de gradient */
  if (mode==1) {
    if(mem) {
      // Warning: niv_max est potentiellement modifie
      if (Mem_pyramide_float(gx,ima[0].nbli,ima[0].nbco,niv_max) == false)
	return false;
      // Warning: niv_max est potentiellement modifie
      if (Mem_pyramide_float(gy,ima[0].nbli,ima[0].nbco,niv_max) == false)
	return false;
    }
    /* calcul des gradients */
    for(i=0;i<nb_niv;i++) {
      
      state = Derivees_float(&ima[i],&gx[i],&gy[i],taille_filtre_gradient);
      if (state == false) {
	// Erreur lors du calcul des gradients
	if (i==0) {
	  // Si 1er niveau on ne peut rien faire, il faut tout arreter
	  return false;
	}
	else {
	  // Pas la peine de continuer a calculer les gradients au niveau du
	  // dessus

	  // Mise a jour du dernier niveau fiable
	  niv_max = i - 1;
	  break; 
	}
      }
    }
  }

  // Mise a jour du nombre de niveaux fiables
  nb_niv = niv_max + 1;
  return true;
}



/*
 * PROCEDURE	: echange_pyr_shorti
 *
 * INPUTS      :
 * ima1           Pointeur sur la pyramide de short.
 * ima2           Pointeur sur la pyramide de short.
 *
 * DESCRIPTION	:
 * La procedure echange les pointeurs des images 1 et 2. L'image 2 devient
 * l'image 1 et l'image 1 devient l'image 2.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void echange_pyr_shorti(TImageShort *ima1, TImageShort *ima2)
{
  short *copie;
  int   i;

  for(i = 0; i < NB_LEVELS_MAX; i++)
    {
      copie = ima1[i].ad;
      ima1[i].ad = ima2[i].ad;
      ima2[i].ad = copie;
    }
}


/*
 * PROCEDURE	: echange_pyr_float
 *
 * INPUTS      :
 * ima1           Pointeur sur la pyramide de float.
 * ima2           Pointeur sur la pyramide de float.
 *
 * DESCRIPTION	:
 * La procedure echange les pointeurs des images 1 et 2. L'image 2 devient
 * l'image 1 et l'image 1 devient l'image 2.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void echange_pyr_float(TImageFloat *ima1, TImageFloat *ima2)
{
  float *copie;
  int i;

  for(i = 0; i < NB_LEVELS_MAX; i++)
    {
      copie = ima1[i].ad;
      ima1[i].ad = ima2[i].ad;
      ima2[i].ad = copie;
    }
}

