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

  DESCRIPTION	: Le fichier contient le calcul de l'ecart type des residuels.
                  Cet ecart type sera utilise pour determiner la valeur
                  finale de la constante C.
                  Le fichier contient egalement le calcul de la constante C
                  a un niveau donne de la pyramide, connaissant la valeur de C
                  au premier et au dernier niveau de la pyramide.

*/

/* Inclusion des fichiers generaux                  */
#include "type.h"
#include "macro.h"

/* Inclusion des prototypes des procedures internes */
#include "variance.h"



/*
 * PROCEDURE	: variance_im_fl
 *
 * INPUTS      :
 * region         Pointeur sur l'image des etiquettes.
 * etiq           Valeur de l'etiquette, c'est-a-dire des points a prendre
 *                en compte dans la region.
 * fen            Fenetre de travail.
 * imdift         Pointeur sur la DFD, la difference d'image deplacee ou
 *                le residuel:
 *                               imdift = I(pi+depl,t+1) - I(pi,t)
 * mode           Mode de calcul de l'ecart type:
 *                  - si 0, estimation classique,
 *                                        /-------------------------\
 *                                       / Somme(ri2)  [Somme(ri)]2
 *                        ecart_type =  /  --------- - ------------
 *                                    \/   Nb_points   [Nb_points]2
 *
 *                  - si 1, estimation robuste par valeur mediane.
 *                        ecart_type = 1.48(med|ri-med(ri)|
 *
 * DESCRIPTION	:
 * La procedure gere le calcul de l'ecart type des residuels "imdift",
 * suivant la valeur de "mode".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

double variance_im_fl(TImageShort *region, int etiq, TWindow *fen,
		      TImageFloat *imdift, int mode)
{
  double 	sommecarre,somme = 0.0,*tab_res;
  int		li,co,nb_points;
  short         *pregion;
  float 	*pdift;

  switch(mode)
    {
    case 0: /* estimation classique */
      sommecarre = 0.0;
      nb_points = 0;
      somme = 0.0;
      for(li=fen->dli;li<fen->fli;li++)
	{
	  pregion = ptrimage(region,li,fen->dco);
	  pdift   = ptrimage_float(imdift,li,fen->dco);
	  for(co=fen->dco;co<fen->fco;co++,pregion++,pdift++)
	    if(*pregion == etiq)
	      {
		nb_points++;
		somme += *pdift;
		sommecarre += (double)*pdift * *pdift;
	      }
	}
      somme = somme/(double)(nb_points);
      somme = sommecarre/(double)(nb_points)-somme*somme;
      somme = sqrt(somme);
      //fprintf(stdout,"Ecart type classique : %7.3f\n",somme);
      break;

    case 1: /* estimation robuste par mediane(mediane) */
      tab_res=(double*)calloc((unsigned)((fen->fco-fen->dco)*
					 (fen->fli-fen->dli)),sizeof(double));
      nb_points = rempli_tab_residuel(tab_res,region,etiq,fen,imdift);
      somme = medmed(tab_res,nb_points);
      free((char *)tab_res);
      //fprintf(stdout,"Deviation mediane absolue : %7.3f\n",somme);
      break;
    }
  return somme;
}



/*
 * PROCEDURE	: rempli_tab_residuel
 *
 * OUTPUT       :
 * tab_res        Tableau des residuels.
 *
 * INPUTS      :
 * zone           Pointeur sur l'image des etiquettes.
 * etiqu          Valeur de l'etiquette, c'est-a-dire des points a prendre
 *                en compte.
 * fen            Fenetre de travail.
 * imdift         Pointeur sur la DFD, la difference d'image deplacee ou
 *                le residuel:
 *                               imdift = I(pi+depl,t+1) - I(pi,t)
 *
 * DESCRIPTION	:
 * La procedure copie le residuel "imdift" dans le tableau "tab_res" et
 * renvoie le nombre de points a utiliser dans le calcul de l'ecart type.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

int rempli_tab_residuel(double *tab_res, TImageShort *zone, int etiqu,
			TWindow *fen, TImageFloat *imdift)
{
  int    	li, co, nb_points;
  float 	*pdt;
  short         *pzo;
  double 	*residu;

  residu = tab_res;
  nb_points = 0;
  for(li=fen->dli;li<fen->fli;li++)
    {
      pzo = ptrimage(zone,li,fen->dco);
      pdt = ptrimage_float(imdift,li,fen->dco);
      for(co = fen->dco;co<fen->fco;co++,pzo++,pdt++)
	if(*pzo == etiqu)
	  {
	    *residu++ = (double)*pdt;
	    nb_points++;
	  }
    }
  return nb_points;
}



/*
 * PROCEDURE	: medmed
 *
 * INPUTS      :
 * tab_residuel   Tableau des residuels I(pi+depl,t+1) - I(pi,t).
 * nb_res         Nombre de points (d'echantillons) pris en compte dans le
 *                calcul de l'ecart type.
 *
 * DESCRIPTION	:
 * La procedure effectue le calcul de l'ecart type = 1.48(med|ri-med(ri)|
 * et renvoie sa valeur. Voir these Odobez page 52.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

double medmed(double *tab_residuel, int nb_res)
{
  double	median;
  int           i;
  double        *res;

  extrait_med_res(tab_residuel,nb_res,&median,nb_res/2);
  res = tab_residuel;
  for(i=0;i<nb_res;i++,res++)
    *res = fabs(*res - median);
  extrait_med_res(tab_residuel,nb_res,&median,nb_res/2);
  return (1.48*median);
}



/*
 * PROCEDURE	    : extrait_med_res
 *
 * INPUTS          :
 * tab_residuel       Tableau des residuels I(pi+depl,t+1) - I(pi,t).
 * nb_res             Nombre de points (d'echantillons) pris en compte dans le
 *                    calcul de l'ecart type.
 *
 * OUTPUTS          :
 * meilleure_mediane  Valeur mediane du residuel ri dans le tableau des
 *                    residuels "tab_residuel".
 * num_median         Position de la valeur mediane dans le tableau des
 *                    residuels "tab_residuel".
 *
 * DESCRIPTION	    :
 * La procedure extrait la valeur mediane des residuels.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void extrait_med_res(double *tab_residuel, int nb_res,
		     double *meilleure_mediane, int num_median)
{
  double	*res,intermediaire;
  int		i;

  while(num_median <= nb_res)
    {
      res = tab_residuel;
      i=1;
      while(i<nb_res)
	{
	  if(*res>*(res+1))
	    {
	      intermediaire = *res;
	      *res = *(res+1);
	      *(res+1) = intermediaire;
	    }
	  i++;
	  res++;
  	}
      nb_res--;
    }
  *meilleure_mediane = tab_residuel[num_median-1];
}



/*
 * PROCEDURE	: det_variance
 *
 * INPUTS      :
 * vi             Variance (ou constante C) au dernier niveau de la pyramide
 *                (niveau "n_niv-1").
 * vf             Variance (ou constante C) a atteindre au niveau 0.
 * n_niv          Nombre de niveaux de la pyramide.
 * niv            Numero du niveau courant.
 *
 * DESCRIPTION	:
 * La procedure retourne la variance courante (ou constante C) a atteindre
 * a un niveau donne, connaissant la valeur de C ("vi") au dernier niveau
 * ("n_niv-1") de la pyramide et la valeur de C ("vf") au niveau final de
 * l'estimation du mouvement.
 *                        vi + beta x (n_niv - niv -1)
 *                    C = ----------------------------
 *                                n_niv - niv
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

double det_variance(double vi, double vf, int n_niv, int niv)
{
  double alpha,beta;

  if(niv==0)
    return vf;
  else
    {
      beta  = ((double)(n_niv)*vf-vi)/(double)(n_niv-1);
      alpha = vi-beta;
      return (alpha/(double)(n_niv-niv) + beta);
    }
}








