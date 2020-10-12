/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef mem_est_h
#define mem_est_h

bool init_mem_est(int nblig, int nbcol, int &niv_final,
		  bool *init_mem_estIsDone,
		  TImageFloat *pyr_fl1, TImageFloat *pyr_fl2,
		  TImageFloat *pyr_fl3, TImageShort *pyr_de_trav);
void efface_memoire_pyr_est(int Num_niv_max, bool *init_mem_estIsDone,
			    TImageFloat *pyr_fl1, TImageFloat *pyr_fl2,
			    TImageFloat *pyr_fl3, TImageShort *pyr_de_trav);

#endif
