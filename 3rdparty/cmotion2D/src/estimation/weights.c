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

DESCRIPTION : Provides the weights management

*/


// General include
#include "type.h"
#include "macro.h"
#include "interplt.h"

#include "acast.h"
#include "famem.h"

// Local include
#include "weights.h"

// Internal include
#include "para_mvt.h"



/*
INPUT		:
region        Address of the estimator support window.
label         Active label in the estimator support window.
compute	Select the initial weights computation. If false, all the
weights are set to 1. If true, the weights are computed
using the robust function.

robust_function   Select the bi-weight function to use (if compute is
positionned at true).
- If 0, Tukey biweight function, wi = ( 1 - (ri/C)2 )2
- If 1, Talwar biweight function, si (ri/C)2 <= 1 wi = 0,
sinon           wi = 1
- If 2, Cauchy biweight function, wi = 1 / ( 1 + (ri/C)2 )
- If 3, Dennis Welsch biweight function, wi = exp( -(ri/C)2 ).

C_value	Value of the C constant used in the robust function.
win		Working window.
imdt          Address of the DFD array: imdt = I(pi+depl,t+1) - I(pi,t)

OUTPUT       :
weights	Address of the array containing the weights.

DESCRIPTION	:
Initialized the weights considering the M-estimator function.

*/

void init_weights(TImageShort *region, int label, bool compute,
    int robust_function, double C_value,
    TWindow win, TImageFloat *imdt, TImageFloat *weights)
{
  float	invnorm  = (float) (1.0 / (C_value * 1.1));
  int	xsize = imdt->nbco;
  int	x, y;

  // reset all the weights map
  set_float(weights->ad, 0.0, weights->nbli * weights->nbco);

  if (compute == false) {
    //
    // Weights are set to 1.0 inside the estimation support - It refers to a
    // classical least mean square case.
    //
    for (y = win.dli; y < win.fli; y++) {
      size_t	off  = MIJ(0, y, 0, xsize);
      short	*pval = &region->ad[off];
      float	*pond = &weights->ad[off];
      for(x = win.dco; x < win.fco; x++) {
        if (pval[x] == label)
          pond[x] = 1.0;
      }
    }
    return;
  }

  for (y = win.dli; y < win.fli; y++) {
    size_t	off  = MIJ(0, y, 0, xsize);
    short	*pval = &region->ad[off];
    float	*src = &imdt->ad[off];
    float	*dst = &weights->ad[off];

    for(x = win.dco; x < win.fco; x++) {
      if (pval[x] == label) {

        float	f = src[x] * invnorm;

        f *= f;	// ^2

        switch (robust_function) {
          case 0 :	// Tukey biweight function
            if (f > 1.f)
              f = 0.f;
            else {
              f = 1.f - f;
              f *= f;	// ^2
            }
            break;

          case 1 :	// Talwar biweight function
            f = (f > 1.f) ? 0.f : 1.f;
            break;

          case 2 :	// Cauchy biweight function
            f = 1.f / (1.f + f);
            break;
          case 3 :	// Dennis Welsch biweight function
            f = (float) exp ((double) -f);
            break;
        }
        dst[x] = f;
      }
    }
  }
}



/*

INPUTS	:
imgx          Address of the spatial horizontal image gradients array.  
imgy          Address of the spatial vertical image gradients array.
imgt          Address of the DFD array: imgt = I(pi+depl,t+1) - I(pi,t).
thet		Address of the parametric motion model parameters array.
model		Id of the motion model.
row_c         Vertical origin of the motion model.
col_c         Horizontal origin of the motion model.
region        Address of the estimator support window array.
label         Active label in the estimator support window.
win           Size of the estimator support window.

OUTPUT        :
weights	Address of the computed weights.

INPUTS      :
robust_function   Select the bi-weight function to use.
- If 0, Tukey biweight function, wi = ( 1 - (ri/C)2 )2
- If 1, Talwar biweight function, si (ri/C)2 <= 1 wi = 0,
sinon           wi = 1
- If 2, Cauchy biweight function, wi = 1 / ( 1 + (ri/C)2 )
- If 3, Dennis Welsch biweight function, wi = exp( -(ri/C)2 ).
C_value	Value of the C constant used in the robust function.

DESCRIPTION	:
Update the weights considering the M-estimator function.

*/
void update_weights(TImageFloat *imgx, TImageFloat *imgy, TImageFloat *imgt,
    double *thet, EIdModel model, double row_c,
    double col_c, TImageShort *region, int label, TWindow win,
    TImageFloat *weights, int robust_function, double C_value)
{
  const double 	coef = (float) (1.0/4.5);
  double	inv_C_value;
  int	        co, li;
  double 	depx, depy;
  short         *pregion;
  float	        *pond, *pgx, *pgy, *pgt;


  switch(robust_function) {
    case 0:
      C_value *= 1.1;
      break;
    case 1:
      C_value *= coef*2.795;
      break;
    case 2:
      C_value *=coef*2.385;
      break;
    case 3:
      C_value *= coef*2.985;
      break;
  }

  inv_C_value = 1.0 / C_value;

  if (model_degree(model)==2) {
    for(li=win.dli;li<win.fli;li++) {
      double d_li = (double) (li) - row_c; 

      double t0 = d_li*d_li;
      double t1 = thet[0] + thet[3]*d_li + t0*thet[8];
      double t2 = thet[7]*d_li;
      double t3 = thet[1] + thet[5]*d_li + t0*thet[11];
      double t4 = thet[10]*d_li;
      double t6 = thet[2] + t2;
      double t7 = thet[4] + t4;

      pregion = ptrimage(region,li,win.dco);
      pond = ptrimage_float(weights,li,win.dco);
      pgx = ptrimage_float(imgx,li,win.dco);
      pgy = ptrimage_float(imgy,li,win.dco);
      pgt = ptrimage_float(imgt,li,win.dco);
      for(co=win.dco;co<win.fco;co++,pregion++,pond++,pgx++,pgy++,pgt++) {
        if(*pregion == label) {
          double	d_co = (double) (co) - col_c;
          double	f;

          //
          //		  double t5 = d_co*d_co;
          //
          //		  depx = t1 + t6*d_co + t5*thet[6];
          //		  depy = t3 + t7*d_co + t5*thet[9];
          //
          depx = t1 + d_co * (t6 + (d_co * thet[6]));
          depy = t3 + d_co * (t7 + (d_co * thet[9]));

          f = (*pgx * depx + *pgy * depy + *pgt + thet[12]) * inv_C_value;
          f *= f;	// ^2

          switch (robust_function) {
            case 0 :	// Tukey biweight function
              if (f > 1.f)
                f = 0.f;
              else {
                f = 1.f - f;
                f *= f;	// ^2
              }
              break;

            case 1 :	// Talwar biweight function
              f = (f > 1.f) ? 0.f : 1.f;
              break;

            case 2 :	// Cauchy biweight function
              f = 1.f / (1.f + f);
              break;
            case 3 :	// Dennis Welsch biweight function
              f = exp (-f);
              break;
          }
          *pond = (float) f;
        }
      }
    }
  }
  else  if (model_degree(model)==1) {
    for(li=win.dli;li<win.fli;li++) {
      double d_li = (double) (li) - row_c;

      double t0 = thet[0] + thet[3]*d_li;
      double t1 = thet[1] + thet[5]*d_li;

      pregion = ptrimage(region,li,win.dco);
      pond = ptrimage_float(weights,li,win.dco);
      pgx = ptrimage_float(imgx,li,win.dco);
      pgy = ptrimage_float(imgy,li,win.dco);
      pgt = ptrimage_float(imgt,li,win.dco);
      for(co=win.dco;co<win.fco;co++,pregion++,pond++,pgx++,pgy++,pgt++) {
        if(*pregion == label) {
          double	d_co = (double) (co) - col_c; // (float) (co - col_c);
          double	f;

          depx = t0 + thet[2] * d_co;
          depy = t1 + thet[4] * d_co;

          f = (*pgx * depx + *pgy * depy + *pgt + thet[12] ) * inv_C_value;
          f *= f;	// ^2

          switch (robust_function) {
            case 0 :	// Tukey biweight function
              if (f > 1.f)
                f = 0.f;
              else {
                f = 1.f - f;
                f *= f;	// ^2
              }
              break;

            case 1 :	// Talwar biweight function
              f = (f > 1.f) ? 0.f : 1.f;
              break;

            case 2 :	// Cauchy biweight function
              f = 1.f / (1.f + f);
              break;
            case 3 :	// Dennis Welsch biweight function
              f = exp (-f);
              break;
          }
          *pond = (float) f;
        }
      }
    }
  }
  else if (model_degree(model)==0) {
    depx = thet[0];
    depy = thet[1];
    for(li=win.dli;li<win.fli;li++) {

      pregion = ptrimage(region,li,win.dco);
      pond = ptrimage_float(weights,li,win.dco);
      pgx = ptrimage_float(imgx,li,win.dco);
      pgy = ptrimage_float(imgy,li,win.dco);
      pgt = ptrimage_float(imgt,li,win.dco);
      for(co=win.dco;co<win.fco;co++,pregion++,pond++,pgx++,pgy++,pgt++) {
        if(*pregion == label) {
          double	f;

          f = (*pgx * depx + *pgy * depy + *pgt + thet[12] ) * inv_C_value;
          f *= f;	// ^2

          switch (robust_function) {
            case 0 :	// Tukey biweight function
              if (f > 1.f)
                f = 0.f;
              else {
                f = 1.f - f;
                f *= f;	// ^2
              }
              break;

            case 1 :	// Talwar biweight function
              f = (f > 1.f) ? 0.f : 1.f;
              break;

            case 2 :	// Cauchy biweight function
              f = 1.f / (1.f + f);
              break;
            case 3 :	// Dennis Welsch biweight function
              f = exp (-f);
              break;
          }
          *pond = (float) f;
        }
      }
    }
  }
}

/*
OUTPUT	:
pweights      Array of coefficients used to compute a linear combination
of the motion parameters.

INPUTS	:
model		Motion model id.
var_light	Indicates if the global illumination parameter is to consider.
width		Window width.
height	Window height.

DESCRIPTION :
Compute the coefficients used to built a linear combination of the motion
parameters. See J.-M. Odobez, P. Bouthemy. Robust multiresolution estimation
of parametric motion models. Journal of Visual Communication and Image
Representation, 6(4):348-365, December 1995 page 351.

*/
void compute_parameter_weights(double *pweights, EIdModel model,
    bool var_light, TWindow win)
{
  double       width, height;
  int          i;

  width  = (double)(win.fco - win.dco);
  height = (double)(win.fli - win.dli);

  for(i=0;i<MAXCOEFSMODEL;i++)
    pweights[i] = 0.0;

  if (var_light)
    pweights[12] = 0.05;

  switch (model_degree(model))
  {
    case 2:
      // Set weights for quadratic terms
      pweights[6] = width *width/12.0;	pweights[9]  = pweights[6];
      pweights[7] = height*width/16.0;	pweights[10] = pweights[7];
      pweights[8] = height*height/12.0;	pweights[11] = pweights[8];
    case 1:
      // Set weights for affine terms
      pweights[2] = width/4.0;	pweights[4]  = pweights[2];
      pweights[3] = height/4.0;	pweights[5]  = pweights[3];
    case 0:
      // Set weights for constant terms
      pweights[0] = 1.0;
      pweights[1] = 1.0;
      break;
  }
}

