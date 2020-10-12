/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
#ifndef estimate_aff_h
#define estimate_aff_h


bool estimate_AFF_TX_DIV(TImageFloat *imgx, TImageFloat *imgy,
			 TImageFloat *imgt, TImageShort *zone_val,
			 int etiq, TWindow win,
			 TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_TR_DIV(TImageFloat *imgx, TImageFloat *imgy,
			 TImageFloat *imgt, TImageShort *zone_val,
			 int etiq, TWindow win,
			 TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_TR_ROT(TImageFloat *imgx, TImageFloat *imgy,
			 TImageFloat *imgt, TImageShort *zone_val,
			 int etiq, TWindow win,
			 TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_TX_NULL(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_TY_NULL(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_DIV_NULL(TImageFloat *imgx, TImageFloat *imgy,
			   TImageFloat *imgt, TImageShort *zone_val,
			   int etiq, TWindow win,
			   TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_ROT_NULL(TImageFloat *imgx, TImageFloat *imgy,
			   TImageFloat *imgt, TImageShort *zone_val,
			   int etiq, TWindow win,
			   TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_HYP1_NULL(TImageFloat *imgx, TImageFloat *imgy,
			    TImageFloat *imgt, TImageShort *zone_val,
			    int etiq, TWindow win,
			    TImageFloat *ima_pond, Para *d_param);
     
bool estimate_AFF_HYP2_NULL(TImageFloat *imgx, TImageFloat *imgy,
			    TImageFloat *imgt, TImageShort *zone_val,
			    int etiq, TWindow win,
			    TImageFloat *ima_pond, Para *d_param);
     
bool estimate_AFF_TR_ROT_DIV(TImageFloat *imgx, TImageFloat *imgy,
			     TImageFloat *imgt, TImageShort *zone_val,
			     int etiq, TWindow win,
			     TImageFloat *ima_pond, Para *d_param);

bool estimate_AFF_COMPLET(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param);

#endif	

     
