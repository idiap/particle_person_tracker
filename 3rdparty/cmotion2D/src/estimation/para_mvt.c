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

  DESCRIPTION	: Le fichier contient la gestion des parametres du modele
                  de mouvement.

*/


#include "type.h"
#include "macro.h"
#include "constant.h"

#include "famem.h"
#include "damem.h"

#include "para_mvt.h"

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

//#define VERBOSE_MODEL



/*
 * PROCEDURE	: Copy_Para
 *
 * INPUT       :
 * th1            Structure Para a copier. (en forme compacte)
 *
 * OUTPUT	:
 * th2            Copie de "th1" (en forme compacte) suivant le mode choisi
 *
 * INPUT       :
 * mode           Mode de copie:
 *      Les anciens modes n'etant pas utiliser, on les a enleves :
 * 	           0 : on copie toute la structure, sauf "backthet".
 *                 1 : on copie toute la structure, sauf "backthet" et "fen".
 * Les modes de copies sont donc :
 *  CP_MDL      : On ne copie que "thet", "li_c", "co_c" et "nb_para", ie le model
 *  CP_THET     : On ne copie que "thet".
 *  CP_AND_INIT_THET : copie toute la structure, mais "th2.thet" est initialise a zero
 *
 * DESCRIPTION	:
 * La procedure copie les parametres "th1" dans "th2" suivant le mode de copie
 * "mode".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */
void copy_para(Para *src, Para *dst, COPY_MODE mode)
{
  int i;
  int nb_para;

  // Copy the residual variance 
  dst->compute_sigma2res  = src->compute_sigma2res;
  dst->sigma2res = src->sigma2res;

  // Copy the covariance matrix 
  dst->compute_covariance = src->compute_covariance;
  memcpy(dst->covariance, src->covariance,
	 MAXCOEFSMODEL * MAXCOEFSMODEL * sizeof(double));

  nb_para = nb_para_max_modele(src->id_model);

  
  switch (mode)  {
  case CP_MDL:

    dst->li_c = src->li_c ;
    dst->co_c = src->co_c ;

    dst->nb_para   = src->nb_para;
    dst->var_light = src->var_light;
    dst->id_model  = src->id_model;

    for (i=0; i < nb_para; i ++)
      dst->thet[i] = src->thet[i];
    for (i=nb_para; i < 12; i ++)
      dst->thet[i] = 0.0;

    if (src->var_light)
      dst->thet[12] = src->thet[12];
    else
      dst->thet[12] = 0.0;

    break;

  case CP_THET:

    for (i=0; i < nb_para; i ++)
      dst->thet[i] = src->thet[i];
    for (i=nb_para; i < 12; i ++)
      dst->thet[i] = 0.0;
    
    if (src->var_light)
      dst->thet[12] = src->thet[12];
    break;

  case CP_AND_INIT_THET:
    dst->n_points = src->n_points ;
    dst->li_c = src->li_c ;
    dst->co_c = src->co_c ;

    dst->nb_para   = src->nb_para;
    dst->var_light = src->var_light;
    dst->id_model  = src->id_model;

    copie_fen(&src->fen,&dst->fen);

    for(i=0;i<MAXCOEFSMODEL;i++)
      dst->thet[i] = 0.0 ;
    break;
  default:
    break;
  }
}


/*
 * PROCEDURE	: CutNKeep_Model
 *
 * INPUT       :
 * par1   structure Para du modele a couper (sous forme compacte)
 * model  partie du modele a conserver  ( 0: translation )
 *
 *
 * OUTPUT	:
 * par1    le modele tronqué (sous forme compacte)
 *
 *
 * DESCRIPTION	:
 * La procedure tronque le modele en ne gardant que la partie desiree (par ex: les termes
 * de translation), mais l'identification du modele reste le meme (id, nb_para, var_light)
 *
 * HISTORIQUE   :
 * 1.30 - 19/04/01
 */

// ATTENTION : cette fonction n'est pas complete !!
// elle ne sert actuellement qu'a l'initialisation des parametres au debut
// de RMRmod(..), ou l'on a pour le moment, seulement besoin de garder
// la partie constante d'un modele.

void cut_para(Para *par, EIdModel model)
{

  int i;
  int nb_para_to_keep;
  
#ifdef VERBOSE_MODEL
  printf("\ncut_para:: model id %d to cut\n", par->id_model);
  aff_Para(par);
#endif

  par->thet[12]=0.0;

  switch (model) {
  case MDL_TX:
  case MDL_TY:
  case MDL_TR:
    // on ne conserve que la partie translation du modele
    nb_para_to_keep = 2;
    for (i=nb_para_to_keep; i < 12; i++)
      par->thet[i]=0.0; 
    break;
    
  default:
    break;
  }

#ifdef VERBOSE_MODEL
  printf("cut_para:: we keep %d\n", model);
  aff_Para(par);
#endif

}



/*
 * PROCEDURE	: copie_fen
 *
 * INPUT       :
 * fen1           Fenetre de travail a copier.
 *
 * OUTPUT	:
 * fen2           Copie de la fenetre de travail "fen1". *
 *
 * DESCRIPTION	:
 * La procedure copie les caracteristiques de la fenetre de travail "fen1"
 * dans "fen2".
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

void copie_fen(TWindow *fen1, TWindow *fen2)
{
  fen2->dli = fen1->dli;
  fen2->fli = fen1->fli;
  fen2->dco = fen1->dco;
  fen2->fco = fen1->fco;
}



/*
  INPUTS      :
  par1           Address of the the first model parameters.
  par2           Address of the the second model parameters.
  coef_pond      Address of the weights affected to the parameters
  mode           Distance computation mode:
                   0 : distance = coef_pond[i] x | par1[i] - par2[2] |
                   1 : distance = coef_pond[i] x | par1[i] |
 
  DESCRIPTION :

  Computes either a norm information about the motion model par1 (mode = 1), or
  a distance information between model 1 and 2 (mode = 2).
 
*/
double distance_para(Para par1, Para par2, const double *coef_pond, int mode)
{
  int	i;
  double  dist = 0.0;

  if (DEBUG_LEVEL1) printf("Begin distance_para()\n");
  if (DEBUG_LEVEL2) {
    printf("Parameters 1: \n");
    aff_theta(par1.thet);
    if (mode == 0) {
      printf("Parameters 2: \n");
      aff_theta(par2.thet);
    }
    printf("Ponderation: \n");
    aff_theta(coef_pond);
  }

  int degree = model_degree(par1.id_model);
  if (DEBUG_LEVEL2) printf("degree: %d\n", degree);
  switch(mode)
  {
  case 0:
    {
      switch(degree)
      {
      case 2:
	for (i=6; i < 12; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i] - par2.thet[i]);
      case 1:
	for (i=2; i < 6; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i] - par2.thet[i]);
      case 0:
	for (i=0; i < 2; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i] - par2.thet[i]);
	break;
      }
      if (par1.var_light)
	dist += coef_pond[12] * fabs(par1.thet[12] - par2.thet[12]);
      break;
    }
  case 1:
    {
      switch(degree)
      {
      case 2:
	for (i=6; i < 12; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i]);
      case 1:
	for (i=2; i < 6; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i]);
      case 0:
	for (i=0; i < 2; i++)
	  dist += coef_pond[i]*fabs(par1.thet[i]);
	break;
      }
      if (par1.var_light)
	dist += coef_pond[12] * fabs(par1.thet[12]);
      
      break;
    }
  }


  if (DEBUG_LEVEL2) printf("distance : %f\n", dist);
  if (DEBUG_LEVEL1) printf("End distance_para()\n");
  return dist;
}



/*
 * PROCEDURE	: test_para
 *
 * INPUTS      :
 * theta_atest    Pointeur sur les parametres a tester, sous forme polynomiale
 *
 * DESCRIPTION	:
 * La procedure teste la valeur des parametres du modele de mouvement.
 * La procedure ne renvoie pas de code retour en cas d'erreur. Elle
 * ne previent de l'erreur que par printf().
 */

bool test_para(double *theta)
{
  int     i;

  /* test de la translation (termes constants du modele) */
  for(i=0;i<2;i++) {
    if (fabs(theta[i]) > 10000.0) {
      fprintf(stderr,
	      "\n\nWarning, translation parameter %d very big: %f\n\n",
	      i, theta[i]);
      return false;
    }
  }

  /* test des autres termes (lin et qua) */
  for(i=2;i<12;i++) {
    if(fabs(theta[i]) > 50.0) {
      fprintf(stderr,
	      "\n\nWarning, linear/quadratic parameter %d very big: %f\n\n",
	      i, theta[i]);
      return false;
    }
  }
  return true;
}


/*
 * PROCEDURE	: Chgt_repere
 *
 * INPUT       :
 * param          Pointeur sur les parametres calcules au centre de gravite
 *                de la fenetre de travail, sous forme compacte
 *
 * OUTPUT       :
 * nouveau_coef   Valeur des parametres "param" dans le nouveau repere.
 *                sous forme compacte
 *
 * INPUTS      :
 * li_c_nouv      Ordonnee du nouveau repere.
 * co_c_nouv      Abscisse du nouveau repere.
 *
 * DESCRIPTION	:
 * La procedure determine la valeur des coefficients de modelisation du
 * champ des vitesses dans un autre repere (co_c_nouv, li_c_nouv).
 *
 * ATTENTION au cas MDL_QUA_PAN_DIV: le chgt de repere est valide si on a en PRE un modele * 4 para. Puis en POST, le modele sera sous une forme a 8 para (meme forme que QUA_2D).
 * Ceci est vrai si le centre du modele change.
 *
 * HISTORIQUE   :
 * 1.00 - 01/01/95 - Original.
 */

bool chgt_repere(Para *param, double ncf[MAXCOEFSMODEL], double li_c_nouv,
		 double co_c_nouv)
{
  double  dep_li,dep_co;
  int	 i;

  for(i=0;i<MAXCOEFSMODEL;i++)
    ncf[i] = 0.0;
  
  dep_li = li_c_nouv - param->li_c;
  dep_co = co_c_nouv - param->co_c;

  switch(model_degree(param->id_model))
  {
  case 2:
    // Quadratic terms
    for(i=6;i<12;i++)
      ncf[i] = param->thet[i];
    // Contribution of the quadratic terms on the affine ones
    ncf[2] = 2.0*param->thet[6] *dep_co +     param->thet[7] *dep_li;
    ncf[3] =     param->thet[7] *dep_co + 2.0*param->thet[8] *dep_li;
    ncf[4] = 2.0*param->thet[9] *dep_co +     param->thet[10]*dep_li;
    ncf[5] =     param->thet[10]*dep_co + 2.0*param->thet[11]*dep_li;

    // Contribution of the quadratic terms on the constant ones
    ncf[0] = param->thet[6]*dep_co*dep_co + param->thet[7]*dep_co*dep_li +
      param->thet[8]*dep_li*dep_li;

    ncf[1] = param->thet[9]*dep_co*dep_co +param->thet[10]*dep_co*dep_li +
      param->thet[11]*dep_li*dep_li;
       
  case 1:
    // Affine terms
    for(i=2;i<6;i++)
      ncf[i] += param->thet[i];

    // Contribution of the affine terms on the constant ones
    ncf[0] += param->thet[2]*dep_co + param->thet[3]*dep_li;
    ncf[1] += param->thet[4]*dep_co + param->thet[5]*dep_li;

  case 0:
    // Constant terms
    ncf[0] += param->thet[0];
    ncf[1] += param->thet[1];
    break;
  }

  ncf[12] = param->thet[12];

  return (test_para(ncf));
}


/*
 * INPUT       :
 * par1           Coefficients du modele de mouvement, sous forme compacte
 *
 * OUTPUT       :
 * par2           Coefficients du modele de mouvement a un autre niveau, sous forme compacte
 *
 * INPUT       :
 * incr           Nombre de niveaux a sauter pour obtenir "par2".
 *                Modification dans la hierarchie pyramidale
 *                  - si incr = 0, on ne fait que recopier thet1 dans thet2
 *                  - si incr > 0, on se deplace de "incr" nombre de niveaux,
 *                    vers les niveaux superieurs de la pyramide.
 *                  - si incr < 0, on se deplace de "incr" nombre de niveaux,
 *                    vers les niveaux inferieurs de la pyramide (ceux ayant
 *                    la resolution la plus fine = image de base).
 *
 * DESCRIPTION	:
 * La procedure modifie les parametres en fonction du niveau ou l'on veut
 * qu'ils se situent dans la pyramide. Seuls les termes constants et
 * quadratiques sont modifies.
 */

void change_level(Para *par1, Para *par2, int incr)
{
  int	         i,j;
  float	 coef = 0.5;

  for (i=0; i<MAXCOEFSMODEL; i ++)
    par2->thet[i] = par1->thet[i];

  par2->li_c = par1->li_c;
  par2->co_c = par1->co_c;

  if(incr>0)	coef = 0.5;
  if(incr<0)	coef = 2.0;
  incr = abs(incr);

  for(i=0;i<incr;i++)
  {
    par2->li_c = par2->li_c * coef;
    par2->co_c = par2->co_c * coef;

    par2->thet[0] *= coef;	/* terme constant */
    par2->thet[1] *= coef;	/* terme constant */

    for(j=6;j<12;j++)
      par2->thet[j] /= coef;	/* termes quadratiques */
  }

  par2->id_model = par1->id_model;
  par2->var_light = par1->var_light;
  par2->nb_para = par1->nb_para;
}


/*
 * PROCEDURE	: mvtcoef_to_poly
 *
 * GLOBAL	:
 * errno	Numero de l'erreur systeme courante (voir perror(3)).
 *
 * INPUT	:
 * c		Tableau des coefficients du modele de mouvement.
 * n		Nombre de parametres du modele de mouvement.
 *
 * OUTPUT	:
 * f		Tableau des coefficients du polynome de la premiere fonction.
 * g		Tableau des coefficients du polynome de la seconde  fonction.
 * dv		Pointeur sur la variation d'intensite.
 *
 * DESCRIPTION	:
 * La procedure convertit jusqu'a MAXCOEFSMODEL coefficients "c" du modele de
 * mouvement en coefficients des fonctions polynomiales "f" et "g" et en
 * variation d'intensite "dv". Le nombre de coefficients convertis et le degre
 * des fonctions polynomiales dependent du nombre de parametres "n" du modele.
 * Les fonctions polynomiales "f" et "g" verifient :
 * x' = f[0] + f[1] * x + f[2] * y + f[3] * x^2 + f[4] * x * y + f[5] * y^2
 * y' = g[0] + g[1] * x + g[2] * y + g[3] * x^2 + g[4] * x * y + g[5] * y^2
 *
 * RETOUR 	:
 * En cas de succes, le nombre de coefficients effectivement convertis est
 * retourne. Sinon, la valeur -1 est retournee et la variable globale "errno"
 * est initialisee pour indiquer la cause de l'erreur.
 *
 * HISTORIQUE	:
 * 1.00 - 01/07/95 - Original.
 */
void mvtcoef_to_poly (const double *thet, bool var_light,
		      double *f, double *g, double *dv)
{

  int i;

  /* initialisation par defaut	*/
  set_double (g, (double) 0.0, 6);
  set_double (f, (double) 0.0, 6);
  *dv = (double) ( var_light ? thet[12] :  0.0);


  // para cst
  f[0] = thet[0];
  g[0] = thet[1];
  // para aff
  for (i=2;i<4;i++) f[i-1] = thet[i];
  for (i=4;i<6;i++) g[i-3] = thet[i];
  // para quad
  for (i=6;i<9;i++)  f[i-3] = thet[i];
  for (i=9;i<12;i++) g[i-6] = thet[i];
}



/*
 * INPUT       :
 * modele         identifiant du modele, et booleen pour la gestion de l'eclairage
 *
 * OUTPUT       : booleen. Renvoie le nb de parametre, SI LE MODELE EXISTE, sinon 0.
 *                comprend la variable d'eclairage.
 *
 * DESCRIPTION	:
 *  La procedure indique si le modele (dans sa version normale ou avec gestion
 *  de la variation d'eclairage) est implemente dans la version de Motion2D
 *
 * HISTORIQUE   :
 * 1.00 - 29/03/01
 */
int Nb_Para_Modele(EIdModel model)
{
  int nb;
  
  switch (model) {
  case MDL_TX: nb = 1; break;
  case MDL_TY: nb = 1; break;
  case MDL_TR: nb = 2; break; 
  case MDL_AFF_TX_DIV: nb = 2; break;
  case MDL_AFF_TR_DIV: nb = 3; break;
  case MDL_AFF_TR_ROT: nb = 3; break;
  case MDL_AFF_TR_ROT_DIV: nb = 4; break;
  case MDL_AFF_TX_NULL: nb = 5; break;
  case MDL_AFF_TY_NULL: nb = 5; break;
  case MDL_AFF_DIV_NULL: nb = 5; break;
  case MDL_AFF_ROT_NULL: nb = 5; break;
  case MDL_AFF_HYP1_NULL: nb = 5; break;
  case MDL_AFF_HYP2_NULL: nb = 5; break;
  case MDL_AFF_COMPLET: nb = 6; break;
  case MDL_QUA_PAN_DIV: nb = 4; break;
  case MDL_QUA_PAN_TILT: nb = 4; break;
  case MDL_QUA_PAN_TILT_DIV: nb = 5; break;
  case MDL_QUA_2D: nb = 8; break;
  case MDL_QUA_COMPLET: nb = 12; break;
  default: nb = 0; break;
  }

  return nb;
}


/*

  INPUT:
  model         model id.
  
  OUTPUT:	2D polynomial motion model degree.
  
  DESCRIPTION :

  Indiquate if the motion model is constant (degree=0), affine (degree=1) or
  quadratic (degree=2).
 
*/
int model_degree(EIdModel model)
{
  int degre;

  switch (model) {
  case MDL_TX: 
  case MDL_TY: 
  case MDL_TR:
    degre = 0; break;
  case MDL_AFF_TX_DIV:
  case MDL_AFF_TR_DIV:
  case MDL_AFF_TR_ROT:
  case MDL_AFF_TR_ROT_DIV: 
  case MDL_AFF_TX_NULL:
  case MDL_AFF_TY_NULL:
  case MDL_AFF_DIV_NULL:
  case MDL_AFF_ROT_NULL:
  case MDL_AFF_HYP1_NULL:
  case MDL_AFF_HYP2_NULL:
  case MDL_AFF_COMPLET:
    degre = 1; break;
  case MDL_QUA_PAN_DIV: 
  case MDL_QUA_PAN_TILT: 
  case MDL_QUA_PAN_TILT_DIV: 
  case MDL_QUA_2D: 
  case MDL_QUA_COMPLET:
    degre = 2; break;
  default:
    degre = -1;
  }

  return degre;
}

/*
  Return the maximal number of parameters for a model.
*/
int nb_para_max_modele(EIdModel model)
{
  switch (model_degree(model))
  {
  case 0: // Constant model
    return 2;
    //    break;
  case 1: // Affine model
    return 6;
    //break;
  case 2: // Quadratic model
    return 12;
    //break;
  }

  return(0);
}

void aff_theta(const double *thet) 
{
  int i;

  printf("theta: \n*> ");
  for (i=0; i<6; i++) printf("%f ",thet[i]);
  printf("\n*> ");
  for (i=6; i<12; i++) printf("%f ",thet[i]);
  printf("[%f]\n",thet[12]);
}

void aff_Para(Para* para) {
  aff_theta(para->thet);
  printf("id: %d\n", para->id_model);
}

