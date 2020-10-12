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
  
  Le fichier contient la gestion de la minimisation aux moidres
  carres ponderes du residuel ri =  X*delta_theta - Y.

*/

#include "type.h"
#include "macro.h"
#include "constant.h"
#include "interplt.h"
#include "daarith.h"
#include "damem.h"
#include "inverse_mat_sym.h"
#include "resoud_mat_sym.h"
#include "covariance.h"

#include "estimate_quad.h"

#include "para_mvt.h"

/* Definition des macro-instructions                */
#define	PERTURBATION	5.0
#define	POND_MINI	0.01

//#define VERBOSE_MODEL

#define DEBUG_LEVEL1 0


/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  fen          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of the motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate_QUA_PAN_DIV(TImageFloat *imgx, TImageFloat *imgy,
			 TImageFloat *imgt, TImageShort *zone_val,
			 int etiq, TWindow win,
			 TImageFloat *ima_pond, Para *d_param)
{
  int           N; // Number of parameters

  N = d_param->nb_para;
  if (d_param->var_light)
    N++;

  int		cnt = 0;		// Pixel counter
  int		nb_pts_utiles = 0;	// Used pixel counter

  double	YTWY	  = 0.0;
  double	YTWXTheta = 0.0;

  double        *A_  = new double [N*N];
  double        **A  = new double* [N];
  double        *B   = new double [N];
  double        *X   = new double [N];
  double        *phi = new double [N];
  double        li_c = d_param->li_c;
  double        co_c = d_param->co_c;

  int		success = 0;	// return value
  int		x;		// column
  int		y;		// row
  int		i, j;
  
  for (i=0; i < N; i ++)
    A[i] = A_ + i*N;

  d_param->sigma2res = 0.0;
  set_double (X, 0.0, N);
  set_double (B, 0.0, N);
  set_double (&A[0][0], 0.0, N * N);

  // Diagonal pertubation, to assume that the matrix would be inversible
  for (i = 0; i < N; i++)
    A[i][i] = PERTURBATION;

  if (d_param->var_light)
    phi[N-1] = 1.0;  // For the global illumination variation parameter 

  for (y = win.dli; y <win.fli; y++)  {
    size_t	off   = MIJ(0, y, 0, ima_pond->nbco);
    short	*pval = &zone_val->ad[off];
    float	*pond = &ima_pond->ad[off];
    float	*pgx  = &imgx->ad[off];
    float	*pgy  = &imgy->ad[off];
    float	*pgt  = &imgt->ad[off];

    double	dy = (double) (y - li_c);

    for(x = win.dco; x < win.fco; x++) {
      double	dpond, dpgt;	// temporary values
      double	dx;

      if ((pval[x]==etiq) && (((pgx[x]!=0.0)||(pgy[x]!=0.0))||(pgt[x]!=0.0))){
	nb_pts_utiles++;
	if (pond[x] > POND_MINI) {
	  cnt++;

      	  dpond = (double) pond[x];
	  dpgt  = (double) pgt[x];
	  dx    = (double) (x - co_c);

	  phi[0] = pgx[x];
	  phi[1] = pgx[x] * dx + pgy[x] * dy;
	  phi[2] = phi[1] * dx;
	  phi[3] = phi[1] * dy;

	  // For the residual variance computation
	  if (d_param->compute_sigma2res)
	    YTWY += dpond * dpgt * dpgt;

	  for (i = 0; i < N; i++)
	  {
	    double	d = (double) phi[i] * dpond;

	    B[i] -= d * dpgt;
	    // Cumulation on a triangle matrix
	    for (j = 0; j <= i; j++)
	      A[i][j] +=  phi[j] * d;
	  }
	}
      }
    }
  }

  // Fill the upper triangle of the matrix
  for (i = 0; i < N; i++)
    for (j = i + 1; j < N; j++)
      A[i][j] = A[j][i];

  d_param->tx_pts = ((double)cnt)/nb_pts_utiles;

  if (cnt <= N * 2) {	// Not enough pixels
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Solve AX = B by exploiting the symetry of the matrix
  success = resoud_mat_sym (&A[0][0], B, X, N, N);

  if (! success) {
    set_double (X, 0.0, N);
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Convert the compact form to the polynomial form
  d_param->thet[0] = X[0];
  d_param->thet[2] = d_param->thet[5]  = X[1];
  d_param->thet[6] = d_param->thet[10] = X[2];
  d_param->thet[7] = d_param->thet[11] = X[3];
  if (d_param->var_light) {
    d_param->thet[12] = X[N-1]; // Illumination variation
  }
  
  if ( ! d_param->compute_sigma2res) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the residual variance
  YTWXTheta = dot_double (B, X, N);
  d_param->sigma2res = (YTWY - YTWXTheta) / (double) cnt;

  if ( ! d_param->compute_covariance) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the covariance matrix
  double *InvA_  = new double [N*N];
  double **InvA  = new double* [N];
  for (i=0; i < N; i ++)
    InvA[i] = InvA_ + i*N;

  success = inverse_mat_sym (&A[0][0], &InvA[0][0], N);
  if (! success) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }
  update_covariance(&InvA[0][0], d_param);

  delete [] A_;
  delete [] A;
  delete [] B;
  delete [] X;
  delete [] phi;
  delete [] InvA_;
  delete [] InvA;

  return true;
}

/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  fen          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of the motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate_QUA_PAN_TILT(TImageFloat *imgx, TImageFloat *imgy,
			  TImageFloat *imgt, TImageShort *zone_val,
			  int etiq, TWindow win,
			  TImageFloat *ima_pond, Para *d_param)
{
  int           N; // Number of parameters

  N = d_param->nb_para;
  if (d_param->var_light)
    N++;

  int		cnt = 0;		// Pixel counter
  int		nb_pts_utiles = 0;	// Used pixel counter

  double	YTWY	  = 0.0;
  double	YTWXTheta = 0.0;

  double        *A_  = new double [N*N];
  double        **A  = new double* [N];
  double        *B   = new double [N];
  double        *X   = new double [N];
  double        *phi = new double [N];
  double        li_c = d_param->li_c;
  double        co_c = d_param->co_c;

  int		success = 0;	// return value
  int		x;		// column
  int		y;		// row
  int		i, j;
  
  for (i=0; i < N; i ++)
    A[i] = A_ + i*N;

  d_param->sigma2res = 0.0;
  set_double (X, 0.0, N);
  set_double (B, 0.0, N);
  set_double (&A[0][0], 0.0, N * N);

  // Diagonal pertubation, to assume that the matrix would be inversible
  for (i = 0; i < N; i++)
    A[i][i] = PERTURBATION;

  if (d_param->var_light)
    phi[N-1] = 1.0;  // For the global illumination variation parameter 

  for (y = win.dli; y <win.fli; y++)  {
    size_t	off   = MIJ(0, y, 0, ima_pond->nbco);
    short	*pval = &zone_val->ad[off];
    float	*pond = &ima_pond->ad[off];
    float	*pgx  = &imgx->ad[off];
    float	*pgy  = &imgy->ad[off];
    float	*pgt  = &imgt->ad[off];

    double	dy = (double) (y - li_c);

    for(x = win.dco; x < win.fco; x++) {
      double	dpond, dpgt;	// temporary values
      double	dx;

      if ((pval[x]==etiq) && (((pgx[x]!=0.0)||(pgy[x]!=0.0))||(pgt[x]!=0.0))){
	nb_pts_utiles++;
	if (pond[x] > POND_MINI) {
	  cnt++;

      	  dpond = (double) pond[x];
	  dpgt  = (double) pgt[x];
	  dx    = (double) (x - co_c);
	  double dx_dy = dx * dy;

	  phi[0] = pgx[x];
	  phi[1] = pgy[x];
	  phi[2] = phi[0] * dx * dx + phi[1] * dx_dy;
	  phi[3] = phi[0] * dx_dy   + phi[1] * dy * dy;

	  // For the residual variance computation
	  if (d_param->compute_sigma2res)
	    YTWY += dpond * dpgt * dpgt;

	  for (i = 0; i < N; i++)
	  {
	    double	d = (double) phi[i] * dpond;

	    B[i] -= d * dpgt;
	    // Cumulation on a triangle matrix
	    for (j = 0; j <= i; j++)
	      A[i][j] +=  phi[j] * d;
	  }
	}
      }
    }
  }


  // Fill the upper triangle of the matrix
  for (i = 0; i < N; i++)
    for (j = i + 1; j < N; j++)
      A[i][j] = A[j][i];

  d_param->tx_pts = ((double)cnt)/nb_pts_utiles;

  if (cnt <= N * 2) {	// Not enough pixels
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Solve AX = B by exploiting the symetry of the matrix
  success = resoud_mat_sym (&A[0][0], B, X, N, N);

  if (! success) {
    set_double (X, 0.0, N);
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Convert the compact form to the polynomial form
  d_param->thet[0] = X[0];
  d_param->thet[1] = X[1];
  d_param->thet[6] = d_param->thet[10] = X[2];
  d_param->thet[7] = d_param->thet[11] = X[3];
  if (d_param->var_light) {
    d_param->thet[12] = X[N-1]; // Illumination variation
  }
  
  if ( ! d_param->compute_sigma2res) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the residual variance
  YTWXTheta = dot_double (B, X, N);
  d_param->sigma2res = (YTWY - YTWXTheta) / (double) cnt;

  if ( ! d_param->compute_covariance) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the covariance matrix
  double *InvA_  = new double [N*N];
  double **InvA  = new double* [N];
  for (i=0; i < N; i ++)
    InvA[i] = InvA_ + i*N;

  success = inverse_mat_sym (&A[0][0], &InvA[0][0], N);
  if (! success) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }
  update_covariance(&InvA[0][0], d_param);

  delete [] A_;
  delete [] A;
  delete [] B;
  delete [] X;
  delete [] phi;
  delete [] InvA_;
  delete [] InvA;

  return true;
}
/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  fen          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of the motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate_QUA_PAN_TILT_DIV(TImageFloat *imgx, TImageFloat *imgy,
			      TImageFloat *imgt, TImageShort *zone_val,
			      int etiq, TWindow win,
			      TImageFloat *ima_pond, Para *d_param)
{
  int           N; // Number of parameters

  N = d_param->nb_para;
  if (d_param->var_light)
    N++;

  int		cnt = 0;		// Pixel counter
  int		nb_pts_utiles = 0;	// Used pixel counter

  double	YTWY	  = 0.0;
  double	YTWXTheta = 0.0;

  double        *A_  = new double [N*N];
  double        **A  = new double* [N];
  double        *B   = new double [N];
  double        *X   = new double [N];
  double        *phi = new double [N];
  double        li_c = d_param->li_c;
  double        co_c = d_param->co_c;

  int		success = 0;	// return value
  int		x;		// column
  int		y;		// row
  int		i, j;
  
  for (i=0; i < N; i ++)
    A[i] = A_ + i*N;

  d_param->sigma2res = 0.0;
  set_double (X, 0.0, N);
  set_double (B, 0.0, N);
  set_double (&A[0][0], 0.0, N * N);

  // Diagonal pertubation, to assume that the matrix would be inversible
  for (i = 0; i < N; i++)
    A[i][i] = PERTURBATION;

  if (d_param->var_light)
    phi[N-1] = 1.0;  // For the global illumination variation parameter 

  for (y = win.dli; y <win.fli; y++)  {
    size_t	off   = MIJ(0, y, 0, ima_pond->nbco);
    short	*pval = &zone_val->ad[off];
    float	*pond = &ima_pond->ad[off];
    float	*pgx  = &imgx->ad[off];
    float	*pgy  = &imgy->ad[off];
    float	*pgt  = &imgt->ad[off];

    double	dy = (double) (y - li_c);

    for(x = win.dco; x < win.fco; x++) {
      double	dpond, dpgt;	// temporary values
      double	dx;

      if ((pval[x]==etiq) && (((pgx[x]!=0.0)||(pgy[x]!=0.0))||(pgt[x]!=0.0))){
	nb_pts_utiles++;
	if (pond[x] > POND_MINI) {
	  cnt++;

      	  dpond = (double) pond[x];
	  dpgt  = (double) pgt[x];
	  dx    = (double) (x - co_c);

	  phi[0] = pgx[x];
	  phi[1] = pgy[x];
	  phi[2] = phi[0] * dx + phi[1] * dy;
	  phi[3] = phi[2] * dx;
	  phi[4] = phi[2] * dy;

	  // For the residual variance computation
	  if (d_param->compute_sigma2res)
	    YTWY += dpond * dpgt * dpgt;

	  for (i = 0; i < N; i++)
	  {
	    double	d = (double) phi[i] * dpond;

	    B[i] -= d * dpgt;
	    // Cumulation on a triangle matrix
	    for (j = 0; j <= i; j++)
	      A[i][j] +=  phi[j] * d;
	  }
	}
      }
    }
  }
  // Fill the upper triangle of the matrix
  for (i = 0; i < N; i++)
    for (j = i + 1; j < N; j++)
      A[i][j] = A[j][i];

  d_param->tx_pts = ((double)cnt)/nb_pts_utiles;

  if (cnt <= N * 2) {	// Not enough pixels
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Solve AX = B by exploiting the symetry of the matrix
  success = resoud_mat_sym (&A[0][0], B, X, N, N);

  if (! success) {
    set_double (X, 0.0, N);
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Convert the compact form to the polynomial form
  d_param->thet[0] = X[0];
  d_param->thet[1] = X[1];
  d_param->thet[2] = d_param->thet[5]  = X[2];
  d_param->thet[6] = d_param->thet[10] = X[3];
  d_param->thet[7] = d_param->thet[11] = X[4];
  if (d_param->var_light) {
    d_param->thet[12] = X[N-1]; // Illumination variation
  }
  
  if ( ! d_param->compute_sigma2res) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the residual variance
  YTWXTheta = dot_double (B, X, N);
  d_param->sigma2res = (YTWY - YTWXTheta) / (double) cnt;

  if ( ! d_param->compute_covariance) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the covariance matrix
  double *InvA_  = new double [N*N];
  double **InvA  = new double* [N];
  for (i=0; i < N; i ++)
    InvA[i] = InvA_ + i*N;

  success = inverse_mat_sym (&A[0][0], &InvA[0][0], N);
  if (! success) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }
  update_covariance(&InvA[0][0], d_param);

  delete [] A_;
  delete [] A;
  delete [] B;
  delete [] X;
  delete [] phi;
  delete [] InvA_;
  delete [] InvA;

  return true;
}


/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  fen          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of the motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate_QUA_2D(TImageFloat *imgx, TImageFloat *imgy,
		    TImageFloat *imgt, TImageShort *zone_val,
		    int etiq, TWindow win,
		    TImageFloat *ima_pond, Para *d_param)
{
  int           N; // Number of parameters

  N = d_param->nb_para;
  if (d_param->var_light)
    N++;

  int		cnt = 0;		// Pixel counter
  int		nb_pts_utiles = 0;	// Used pixel counter

  double	YTWY	  = 0.0;
  double	YTWXTheta = 0.0;

  double        *A_  = new double [N*N];
  double        **A  = new double* [N];
  double        *B   = new double [N];
  double        *X   = new double [N];
  double        *phi = new double [N];
  double        li_c = d_param->li_c;
  double        co_c = d_param->co_c;

  int		success = 0;	// return value
  int		x;		// column
  int		y;		// row
  int		i, j;
  
  for (i=0; i < N; i ++)
    A[i] = A_ + i*N;

  d_param->sigma2res = 0.0;
  set_double (X, 0.0, N);
  set_double (B, 0.0, N);
  set_double (&A[0][0], 0.0, N * N);

  // Diagonal pertubation, to assume that the matrix would be inversible
  for (i = 0; i < N; i++)
    A[i][i] = PERTURBATION;

  if (d_param->var_light)
    phi[N-1] = 1.0;  // For the global illumination variation parameter 

  for (y = win.dli; y <win.fli; y++) {
    size_t	off   = MIJ(0, y, 0, ima_pond->nbco);
    short	*pval = &zone_val->ad[off];
    float	*pond = &ima_pond->ad[off];
    float	*pgx  = &imgx->ad[off];
    float	*pgy  = &imgy->ad[off];
    float	*pgt  = &imgt->ad[off];

    double	dy = (double) (y - li_c);

    for(x = win.dco; x < win.fco; x++) {
      double	dpond, dpgt;	// temporary values
      double	dx;

      if ((pval[x]==etiq) && (((pgx[x]!=0.0)||(pgy[x]!=0.0))||(pgt[x]!=0.0))){
	nb_pts_utiles++;
	if (pond[x] > POND_MINI) {
	  cnt++;

	  dpond = (double) pond[x];
	  dpgt  = (double) pgt[x];
	  dx    = (double) (x - co_c);

	  phi[0] = pgx[x];
	  phi[1] = pgy[x];
	  phi[2] = phi[0] * dx;
	  phi[3] = phi[0] * dy;
	  phi[4] = phi[1] * dx;
	  phi[5] = phi[1] * dy;
	  phi[6] = (phi[2] + phi[5]) * dx;
	  phi[7] = (phi[2] + phi[5]) * dy;

	  // For the residual variance computation
	  if (d_param->compute_sigma2res)
	    YTWY += dpond * dpgt * dpgt;

	  for (i = 0; i < N; i++)   {
	    double	d = (double) phi[i] * dpond;

	    B[i] -= d * dpgt;
	    // Cumulation on a triangle matrix
	    for (j = 0; j <= i; j++)
	      A[i][j] +=  phi[j] * d;
	  }
	}
      }
    }
  }

  // Fill the upper triangle of the matrix
  for (i = 0; i < N; i++)
    for (j = i + 1; j < N; j++)
      A[i][j] = A[j][i];

  d_param->tx_pts = ((double)cnt)/nb_pts_utiles;

  if (cnt <= N * 2) {	// Not enough pixels
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Solve AX = B by exploiting the symetry of the matrix
  success = resoud_mat_sym (&A[0][0], B, X, N, N);

  if (! success) {
    set_double (X, 0.0, N);
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Convert the compact form to the polynomial form
  d_param->thet[0] = X[0];
  d_param->thet[1] = X[1];
  d_param->thet[2] = X[2];
  d_param->thet[3] = X[3];
  d_param->thet[4] = X[4];
  d_param->thet[5] = X[5];
  d_param->thet[6] = d_param->thet[10] = X[6];
  d_param->thet[7] = d_param->thet[11] = X[7];
  if (d_param->var_light) {
    d_param->thet[12] = X[N-1]; // Illumination variation
  }
  
  if ( ! d_param->compute_sigma2res) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the residual variance
  YTWXTheta = dot_double (B, X, N);
  d_param->sigma2res = (YTWY - YTWXTheta) / (double) cnt;

  if ( ! d_param->compute_covariance) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the covariance matrix
  double *InvA_  = new double [N*N];
  double **InvA  = new double* [N];
  for (i=0; i < N; i ++)
    InvA[i] = InvA_ + i*N;

  success = inverse_mat_sym (&A[0][0], &InvA[0][0], N);
  if (! success) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }
  update_covariance(&InvA[0][0], d_param);

  delete [] A_;
  delete [] A;
  delete [] B;
  delete [] X;
  delete [] phi;
  delete [] InvA_;
  delete [] InvA;

  return true;
}

/*

  INPUTS      :
  imgx         Spatial gradients under x.
  imgy         Spatial gradients under y.
  imgt         Temporal gradient (DFD): imgt = I(pi+depl,t+1) - I(pi,t)
  zone_val     Estimation support.
  etiq	       Support value to take into consderation.
  fen          Work window.
  ima_pond     Ponderation.

  OUTPUT      :
  d_param      Estimated motion model in the polynomial form.
 
  DESCRIPTION :
  Compute a robust estimation of the motion model.

  RETURN      :
  The number of pixels used for the computation.
  
*/
bool estimate_QUA_COMPLET(TImageFloat *imgx, TImageFloat *imgy,
			 TImageFloat *imgt, TImageShort *zone_val,
			 int etiq, TWindow win,
			 TImageFloat *ima_pond, Para *d_param)
{
  int           N; // Number of parameters

  N = d_param->nb_para;
  if (d_param->var_light)
    N++;

  int		cnt = 0;		// Pixel counter
  int		nb_pts_utiles = 0;	// Used pixel counter

  double	YTWY	  = 0.0;
  double	YTWXTheta = 0.0;

  double        *A_  = new double [N*N];
  double        **A  = new double* [N];
  double        *B   = new double [N];
  double        *X   = new double [N];
  double        *phi = new double [N];
  double        li_c = d_param->li_c;
  double        co_c = d_param->co_c;

  int		success = 0;	// return value
  int		x;		// column
  int		y;		// row
  int		i, j;
  
  for (i=0; i < N; i ++)
    A[i] = A_ + i*N;

  d_param->sigma2res = 0.0;
  set_double (X, 0.0, N);
  set_double (B, 0.0, N);
  set_double (&A[0][0], 0.0, N * N);

  // Diagonal pertubation, to assume that the matrix would be inversible
  for (i = 0; i < N; i++)
    A[i][i] = PERTURBATION;

  if (d_param->var_light)
    phi[N-1] = 1.0;  // For the global illumination variation parameter 

  for (y = win.dli; y <win.fli; y++)  {
    size_t	off   = MIJ(0, y, 0, ima_pond->nbco);
    short	*pval = &zone_val->ad[off];
    float	*pond = &ima_pond->ad[off];
    float	*pgx  = &imgx->ad[off];
    float	*pgy  = &imgy->ad[off];
    float	*pgt  = &imgt->ad[off];

    double	dy = (double) (y - li_c);

    for(x = win.dco; x < win.fco; x++) {
      double	dpond, dpgt;	// temporary values
      double	dx;

      if ((pval[x]==etiq) && (((pgx[x]!=0.0)||(pgy[x]!=0.0))||(pgt[x]!=0.0))){
	nb_pts_utiles++;
	if (pond[x] > POND_MINI) {
	  cnt++;

	  dpond = (double) pond[x];
	  dpgt  = (double) pgt[x];
	  dx    = (double) (x - co_c);

  	  phi[0]  = pgx[x];
	  phi[1]  = pgy[x];
	  phi[2]  = phi[0] * dx;
	  phi[3]  = phi[0] * dy;
	  phi[4]  = phi[1] * dx;
	  phi[5]  = phi[1] * dy;
	  phi[6]  = phi[2] * dx;
	  phi[7]  = phi[2] * dy;
	  phi[8]  = phi[3] * dy;
	  phi[9]  = phi[4] * dx;
	  phi[10] = phi[4] * dy;
	  phi[11] = phi[5] * dy;

	  // For the residual variance computation
	  if (d_param->compute_sigma2res)
	    YTWY += dpond * dpgt * dpgt;

	  for (i = 0; i < N; i++)  {
	    double	d = (double) phi[i] * dpond;

	    B[i] -= d * dpgt;
	    // Cumulation on a triangle matrix
	    for (j = 0; j <= i; j++)
	      A[i][j] +=  phi[j] * d;
	  }
	}
      }
    }
  }

  // Fill the upper triangle of the matrix
  for (i = 0; i < N; i++)
    for (j = i + 1; j < N; j++)
      A[i][j] = A[j][i];

  d_param->tx_pts = ((double)cnt)/nb_pts_utiles;

  if (cnt <= N * 2) {	// Not enough pixels
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Solve AX = B by exploiting the symetry of the matrix
  success = resoud_mat_sym (&A[0][0], B, X, N, N);

  if (! success) {
    set_double (X, 0.0, N);
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }

  // Convert the compact form to the polynomial form
  if (d_param->var_light) {
    for (i=0; i < N-1; i ++)
      d_param->thet[i] = X[i];
    d_param->thet[12] = X[N-1]; // Illumination variation
  }
  else {
    for (i=0; i < N; i ++)
      d_param->thet[i] = X[i];
  }
 
  if ( ! d_param->compute_sigma2res) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the residual variance
  YTWXTheta = dot_double (B, X, N);
  d_param->sigma2res = (YTWY - YTWXTheta) / (double) cnt;

  if ( ! d_param->compute_covariance) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return true;
  }

  // Compute the covariance matrix
  double *InvA_  = new double [N*N];
  double **InvA  = new double* [N];
  for (i=0; i < N; i ++)
    InvA[i] = InvA_ + i*N;

  success = inverse_mat_sym (&A[0][0], &InvA[0][0], N);
  if (! success) {
    delete [] A_;
    delete [] A;
    delete [] B;
    delete [] X;
    delete [] phi;
    return false;
  }
  update_covariance(&InvA[0][0], d_param);

  delete [] A_;
  delete [] A;
  delete [] B;
  delete [] X;
  delete [] phi;
  delete [] InvA_;
  delete [] InvA;

  return true;
}


