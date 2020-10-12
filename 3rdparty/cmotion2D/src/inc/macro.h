/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef  macro_h
#define  macro_h

/* Inclusion des fichiers standards                 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>

/* Definition des macro instructions                */
#define Max(A,B) (((A) > (B) )? (A) : (B))
#define Min(A,B) (((A) < (B) )? (A) : (B))
#define round(vf) ( ((vf)<0)? (int)( (vf)-0.5):(int)( (vf)+0.5))
#define	Norme2(x,x1,y,y1) sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1) )

#define image_l(Im,l) 		*(Im->ad + (l) * (Im->nbco))
#define image(Im,l,c) 		*(Im->ad + (l) * (Im->nbco) + c)
#define ptrimage_l(Im,l) 	(short *)(Im->ad + (l) * (Im->nbco))
#define ptrimage(Im,l,c) 	(short *)(Im->ad + (l) * (Im->nbco) + c)
#define ptrimage_float(Im,l,c) 	(float *)(Im->ad + (l) * (Im->nbco) + c)
#define in_ima(ima,li,co)	((((int)li)>=0) && (((int)co)>=0) && (((int)li)<(ima->nbli-1)) && (((int)co)<(ima->nbco-1)))

#define	M_IJ(m,col,i,j)		((m) + ((i) * (col)) + (j))

#define det_fen(f1,f2,niv,ech)	ech=01<<(niv); f2.dli=f1->dli/ech+1;\
					f2.dco=f1->dco/ech+1;\
					f2.fli=f1->fli/ech;\
					f2.fco=f1->fco/ech

#define cp_fen(f1,f2) f2.dli = f1.dli; f2.dco = f1.dco; \
                      f2.fli = f1.fli; f2.fco = f1.fco

#define rempli_im(ima,nom,nbli,nbco) sprintf(ima.nom,"%s",nom); \
                                     ima.nbli=nbli; ima.nbco = nbco; \
                                     *ima.directory='\0'; \
                                     *ima.extension='\0', ima.numero =-1; \
                                     ima.nboct = 1

#define rempli_im_p(ima,nom,nbli,nbco) sprintf(ima->nom,"%s",nom); \
                                       ima->nbli=nbli; ima->nbco = nbco; \
                                       *ima->directory='\0'; \
                                       *ima->extension='\0', ima->numero =-1; \
                                       ima->nboct = 1


#define get_int(pentier,dummy) scanf("%d",(pentier)); \
			  while( (dummy=getchar()) != EOF && (dummy!='\n'))

#define get_float(pfloat,dummy) scanf("%f",(pfloat)); \
			  while( (dummy=getchar()) != EOF && (dummy!='\n'))

#define get_double(pdouble,dummy) scanf("%lf",(pdouble)); \
			  while( (dummy=getchar()) != EOF && (dummy!='\n'))

#define get_string(pchar,dummy) scanf("%s",(pchar)); \
			  while( (dummy=getchar()) != EOF && (dummy!='\n'))


#endif
