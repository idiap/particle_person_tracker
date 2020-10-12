/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/


#ifndef para_mvt_h
#define para_mvt_h

void copy_para(Para *src, Para *dst, COPY_MODE mode);
//void copie_para(Para *th1, Para *th2, int mode);
void cut_para( Para *par, EIdModel model);
void copie_fen(TWindow *fen1, TWindow *fen2);
double distance_para(Para par1, Para par2, const double *coef_pond, int mode);
bool test_para( double *theta);
bool chgt_repere(Para *param, double ncf[MAXCOEFSMODEL], double li_c_nouv,
		 double co_c_nouv);
void change_level(Para *par1, Para *par2, int incr);

void mvtcoef_to_poly (const double *thet, bool var_light,
		      double *f, double *g, double *dv);


int Nb_Para_Modele(EIdModel model);
int model_degree(EIdModel model);
int nb_para_max_modele(EIdModel model);

void aff_theta(const double *thet);
void aff_Para(Para *para);

#endif	/* para_mvt_h */

