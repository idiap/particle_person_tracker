/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef type_h
#define type_h

/* Inclusion des fichiers standards                 */
#include <stdio.h>
#include "constant.h"
#include <cmotion2d/Motion2D.h>


/* Declaration des structures                       */

/*
 * Structure Look Up Table utilisee pour le changement lineaire de niveaux
 * de gris lors du seuillage de l'image.
 */
typedef struct{
  int     flag;          /* si 0; la LUT n'existe pas, sinon 1     */
  int     s1;            /* valeur du seuil inferieur              */
  int     s2;            /* valeur du seuil superieur              */
  int     v1;            /* niveau de gris pour le seuil inferieur */
  int     v2;            /* niveau de gris pour le seuil superieur */
  short  *ad;            /* pointeur sur le debut de la LUT        */
} LUT;



/*
 * Structure FILTRE_GAUSS utilisee pour sauvegarder les coefficients du
 * filtre gaussien utilise pour filtrer l'image avant sous-echantillonnage.
 */
typedef struct{
  int     flag;          /* si 0; le filtre n'existe pas, sinon 1          */
  double  variance;      /* variance du filtre gaussien                    */
  int     taille;        /* taille du filtre gaussien: taille = 2*nindex+1 */
  int     nindex;
  double *ad_coef;       /* pointeur sur les coefficients du filtre        */
  double *ad_pond;       /* pointeur sur la ponderation du filtre          */
} FILTRE_GAUSS;

/*
 * structure TWindow permettant de definir les coordonnees d'une fenetre
 * dans l'image.
 */
typedef struct {
  int	dco;  	     /* origine en x de la fenetre       */
  int	dli;	     /* origine en y de la fenetre       */
  int	fco;	     /* ligne de fin; Ce numero de ligne */
                     /* n'appartient pas a la fenetre.   */
  int	fli;	     /* colonne de fin; celle_ci         */
                     /* n'appartient pas a la fenetre    */
} TWindow;


/*
  CP_MDL      : On ne copie que "thet", "li_c", "co_c" et "nb_para", ie le model
  CP_THET     : On ne copie que "thet".
  CP_AND_INIT_THET : copie toute la structure, mais "thet" est initialise a zero
*/
typedef enum { CP_MDL, CP_THET, CP_AND_INIT_THET } COPY_MODE;

/*
 * structure Para. Contient la valeur estimee des parametres du modele de
 * mouvement.
 */
typedef struct {
  double    thet[13];     /* parametres du modele */
  double    li_c,co_c;    /* coordonnees du point ou est calcule le modele */
  int	    n_points;     /* nombre de points pris en compte dans le calcul */
  int	    nb_para;      /* nombre de parametres du modele */

  bool      var_light;
  EIdModel  id_model;

  TWindow   fen;       /* fenetre dans laquelle les parametres sont calcules */
  bool      compute_sigma2res;
  double    sigma2res;	/* Variance du residu.	*/
  double    tx_pts;	/* Proportion de points conformes.	*/
  bool      compute_covariance; //  If true compute the covariance matrix
  double    covariance[MAXCOEFSMODEL * MAXCOEFSMODEL];	// Covariance matrix
} Para;


/*
 * structure caracterisant les offsets pour effectuer un balayage centre
 * entre 3 images. Ces offsets permettent de centrer les 3 images.
 */
typedef	struct {
  int	bl;          /* nombre minimal de lignes des images 1,2 ou 3       */
  int	bc;          /* nombre minimal de colonnes des images 1,2 ou 3     */
  int	offset1l;    /* offset ligne de l'image 1 permettant le centrage   */
  int	offset1c;    /* offset colonne de l'image 1 permettant le centrage */
  int	offset2l;    /* offset ligne de l'image 2 permettant le centrage   */
  int	offset2c;    /* offset colonne de l'image 2 permettant le centrage */
  int	offset3l;    /* offset ligne de l'image 3 permettant le centrage   */
  int	offset3c;    /* offset colonne de l'image 3 permettant le centrage */
} BALAYAGE3;


#endif	/* type_h */
