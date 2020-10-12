/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef constant_h
#define constant_h

/* nombre maximum de coefficients du modele de mouvement	*/
#define	MAXCOEFSMODEL	13

/* Definition des macro instructions */
#ifndef	FILENAME_MAX
#  define	FILENAME_MAX	1024	/* Taille maxi des noms de fichier */
#endif

#ifndef MAXPATHLEN
#  define MAXPATHLEN		1024
#endif

#define EPSILON		1.E-10

#define NB_LEVELS_MAX	10   /* Nombre de niveaux maximum dans les    */
				  /* schemas multigrilles et multiechelles */
#define ETI_PIX_FIX	0    /* pixel dit fixe                        */
#define ETI_PIX_MOB	1    /* valeur diff‰rente de Etiqfond et      */
                                  /* Etiqelim                              */

#define ETIQFOND	180  /* <>0 : ‰tiquette des r‰gions € prendre */
                                  /* en consid‰ration pour l'estimation du */
                                  /* mouvement                             */
#define ETIQELIM	80   /* <>0 : ‰tiquette des r‰gions € ne pas  */
                                  /* prendre en consid‰ration              */

#define NO_ERROR	0
#define ERROR		1
#define OUI		1
#define NON		0
#define FALSE		0
#define TRUE		1
#define CREE		10
#define ACTIF		1
#define NON_ACTIF	0
#define NON_UTILISE	-1
#define UN		01


/* Type de mouvement contraint.	*/
enum {
  NONE	= 0,
  TX	= 1,
  TY	= 2,
  DIV	= 3,
  ROT	= 4,
  H1	= 5,
  H2	= 6,
  ALL	= 7
};


#endif	/* constant_h */
