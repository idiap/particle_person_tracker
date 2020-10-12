/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include "type.h"
#include "para_mvt.h"
#include "covariance.h"

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

/*

  Compute the covariance matrix of the estimated model.

  \param m:  \f$ [X^T W X]^{-1} \f$ matrix. This matrix is square and has a
  dimension of N * N, with N the number of parameters of the model. This number
  takes eventually into account the global illumination parameter.

  
  The covariance matrix associated to the estimated parameters is than equal
  to: \f$ \hat{\sigma^2} [X^T W X]^{-1} \f$

  This matrix is than converted from the compact form N * N (with N equal to
  the number of parameters of the estimated motion model), to the polynomial
  form MAXCOEFSMODEL * MAXCOEFSMODEL.

*/
bool update_covariance(double *m, Para *param)
{
  double *cov        = param->covariance;
  int    cov_size    = MAXCOEFSMODEL;
  double var_res     = param->sigma2res;
  int    nb_para     = param->nb_para;
  int    id_model    = param->id_model;
  int    m_size      = nb_para;

  int i_c_c1; // index of c1 parameter in covariance matrix
  int i_c_c2; // index of c2 parameter in covariance matrix
  int i_c_a1; // index of a1 parameter in covariance matrix
  int i_c_a2; // index of a2 parameter in covariance matrix
  int i_c_a3; // index of a3 parameter in covariance matrix
  int i_c_a4; // index of a2 parameter in covariance matrix
  int i_c_q1; // index of q1 parameter in covariance matrix
  int i_c_q2; // index of q2 parameter in covariance matrix
  int i_c_l;  // index of light parameter in covariance matrix

  int i_m_c1; // index of c1 parameter in m (input)  matrix
  int i_m_c2; // index of c2 parameter in m (input)  matrix
  int i_m_a1; // index of a1 parameter in m (input)  matrix
  int i_m_a2; // index of a2 parameter in m (input)  matrix
  int i_m_a3; // index of a3 parameter in m (input)  matrix
  int i_m_a4; // index of a4 parameter in m (input)  matrix
  int i_m_q1; // index of q1 parameter in m (input)  matrix
  int i_m_q2; // index of q2 parameter in m (input)  matrix
  int i_m_l;  // index of light parameter in m (input)  matrix

  if (param->var_light)
    m_size ++;

  int i, j;
  switch (id_model) { 
  case MDL_TY: { // parameter c2: index m: 0 index cov: 1
    i_c_c2 = 1; // index of c2 parameter in covariance matrix
    i_m_c2 = 0; // index of c2 parameter in input matrix

    // Update the covariance matrix considering the motion model parameters
    cov[i_c_c2 * cov_size + i_c_c2] = var_res * m[i_m_c2];
    
    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      // light parameter: index m: 1=nb_para, index cov: (cov_size-1)
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      cov[i_c_l * cov_size + i_m_l] = var_res * m[i_m_l * m_size];
      cov[i_m_l * cov_size + i_c_l] = var_res * m[i_m_l];
      
      // The diagonal
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }
  
  case MDL_AFF_TX_DIV: {// parameter c1, a1
    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_c_a1 = 2; // index of a1 parameter in covariance matrix
    i_m_a1 = 1; // index of a1 parameter in input matrix

    // Update the covariance matrix considering the motion model parameters
    cov[i_c_c1] = var_res * m[i_m_c1];
    cov[i_c_c1 * cov_size + i_c_a1] = var_res * m[i_m_c1 * m_size + i_m_a1];

    cov[i_c_a1 * cov_size + i_c_c1] = var_res * m[i_m_a1 * m_size + i_m_c1];
    cov[i_c_a1 * cov_size + i_c_a1] = var_res * m[i_m_a1 * m_size + i_m_a1];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      cov[i_c_l*cov_size + i_c_c1] = var_res * m[i_m_l * m_size + i_m_c1];
      cov[i_c_l*cov_size + i_c_a1] = var_res * m[i_m_l * m_size + i_m_a1];
      cov[i_c_c1*cov_size + i_c_l] = var_res * m[i_m_c1 * m_size + i_m_l];
      cov[i_c_a1*cov_size + i_c_l] = var_res * m[i_m_a1 * m_size + i_m_l];
     
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }
  

  case MDL_TX: // parameter c1
  case MDL_TR: // parameter c1, c2
  case MDL_AFF_TR_DIV: // parameter c1, c2, a1
  case MDL_AFF_DIV_NULL: // parameter c1, c2, a1, a2, a3
  case MDL_AFF_ROT_NULL: // parameter c1, c2, a1, a2, a3
  case MDL_AFF_HYP1_NULL: // parameter c1, c2, a1, a2, a3
  case MDL_AFF_TR_ROT: // parameter c1, c2, a1
  case MDL_AFF_TR_ROT_DIV: // parameter c1, c2, a1, a2
  case MDL_AFF_COMPLET: // parameter c1, c2, a1, a2, a3, a4
  case MDL_QUA_2D: // parameter c1, c2, a1, a2, a3, a4, q1, q2
  case MDL_QUA_COMPLET: {// parameter c1, c2, a1, a2, a3, a4, q1, ..., q6
    // Update the covariance matrix considering the motion model parameters
    for (i=0; i < nb_para; i++)
      for (j=0; j < nb_para; j++)
	cov[i * cov_size + j] = var_res * m[i * m_size + j];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      for (i=0; i < nb_para; i++) {
	cov[i_c_l*cov_size + i] = var_res * m[i_m_l * m_size + i];
	cov[i*cov_size + i_c_l] = var_res * m[i * m_size + i_m_l];
      }
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }

  case MDL_AFF_TX_NULL: {// parameter c2, a1, a2, a3, a4
    // Update the covariance matrix considering the motion model parameters
    int offset = 1;
    for (i=0; i < nb_para; i++)
      for (j=0; j < nb_para; j++)
	cov[(i+offset) * cov_size + (j+offset)] = var_res * m[i * m_size + j];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      for (i=0; i < nb_para; i++) {
	cov[i_c_l*cov_size + (i+offset)] = var_res * m[i_m_l * m_size + i];
	cov[(i+offset)*cov_size + i_c_l] = var_res * m[i * m_size + i_m_l];
      }
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }

  case MDL_AFF_TY_NULL: {// parameter c1, a1, a2, a3, a4
    // Update the covariance matrix considering the motion model parameters
    int size = 4; // concern a1, a2, a3, a4
    int c_offset = 2; // position for a1 in cov matrix
    int m_offset = 1; // position for a1 in input matrix
    for (i=0; i < size; i++)
      for (j=0; j < size; j++)
	cov[(i+c_offset) * cov_size + (j+c_offset)]
	  = var_res * m[(i+m_offset) * m_size + (j+m_offset)];
    
    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_c_a1 = 2; // index of a1 parameter in covariance matrix
    i_c_a2 = 3; // index of a2 parameter in covariance matrix
    i_c_a3 = 4; // index of a3 parameter in covariance matrix
    i_c_a4 = 5; // index of a4 parameter in covariance matrix

    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_m_a1 = 1; // index of a1 parameter in input matrix
    i_m_a2 = 2; // index of a2 parameter in input matrix
    i_m_a3 = 3; // index of a3 parameter in input matrix
    i_m_a4 = 4; // index of a4 parameter in input matrix

    cov[i_c_c1] = var_res * m[i_m_c1];
  
    cov[i_c_c1 * cov_size + i_c_a1] = var_res * m[i_m_a4 * m_size + i_m_a1];
    cov[i_c_c1 * cov_size + i_c_a2] = var_res * m[i_m_a4 * m_size + i_m_a2];
    cov[i_c_c1 * cov_size + i_c_a3] = var_res * m[i_m_a4 * m_size + i_m_a3];
    cov[i_c_c1 * cov_size + i_c_a4] = var_res * m[i_m_a4 * m_size + i_m_a4];

    cov[i_c_a1 * cov_size + i_c_c1] = var_res * m[i_m_a1 * m_size + i_m_c1];
    cov[i_c_a2 * cov_size + i_c_c1] = var_res * m[i_m_a2 * m_size + i_m_c1];
    cov[i_c_a3 * cov_size + i_c_c1] = var_res * m[i_m_a3 * m_size + i_m_c1];
    cov[i_c_a4 * cov_size + i_c_c1] = var_res * m[i_m_a4 * m_size + i_m_c1];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      for (i=0; i < size; i++) {
	cov[i_c_l*cov_size + (i+c_offset)]
	  = var_res * m[i_m_l * m_size + (i+m_offset)];
	cov[(i+c_offset)*cov_size + i_c_l]
	  = var_res * m[(i+m_offset) * m_size + i_m_l];
      }
      
      cov[i_c_l*cov_size + i_c_c1] = var_res * m[i_m_l * m_size + i_m_c1];
      cov[i_c_c1*cov_size + i_c_l] = var_res * m[i_c_c1 * m_size + i_m_l];

      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }

  case MDL_AFF_HYP2_NULL: {// parameter c1, c2, a1, a2, a4
    // Update the covariance matrix considering the motion model parameters
    int size = 4; // concern c1, c2, a1, a2
    for (i=0; i < size; i++)
      for (j=0; j < size; j++)
	cov[i * cov_size + j] = var_res * m[i * m_size + j];

    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_c_c2 = 1; // index of c2 parameter in covariance matrix
    i_c_a1 = 2; // index of a1 parameter in covariance matrix
    i_c_a2 = 3; // index of a2 parameter in covariance matrix
    i_c_a4 = 5; // index of a4 parameter in covariance matrix

    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_m_c2 = 1; // index of c2 parameter in input matrix
    i_m_a1 = 2; // index of a1 parameter in input matrix
    i_m_a2 = 3; // index of a2 parameter in input matrix
    i_m_a4 = 4; // index of a4 parameter in input matrix

    cov[i_c_a4 * cov_size + i_c_c1] = var_res * m[i_m_a4 * m_size + i_m_c1];
    cov[i_c_a4 * cov_size + i_c_c2] = var_res * m[i_m_a4 * m_size + i_m_c2];
    cov[i_c_a4 * cov_size + i_c_a1] = var_res * m[i_m_a4 * m_size + i_m_a1];
    cov[i_c_a4 * cov_size + i_c_a2] = var_res * m[i_m_a4 * m_size + i_m_a2];

    cov[i_c_c1 * cov_size + i_c_a4] = var_res * m[i_m_c1 * m_size + i_m_a4];
    cov[i_c_c2 * cov_size + i_c_a4] = var_res * m[i_m_c2 * m_size + i_m_a4];
    cov[i_c_a1 * cov_size + i_c_a4] = var_res * m[i_m_a1 * m_size + i_m_a4];
    cov[i_c_a2 * cov_size + i_c_a4] = var_res * m[i_m_a2 * m_size + i_m_a4];

    cov[i_c_a4 * cov_size + i_c_a4] = var_res * m[i_m_a4 * m_size + i_m_a4];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = size;      // index of light parameter in covariance matrix
      for (i=0; i < size; i++) {
	cov[i_c_l*cov_size + i] = var_res * m[i_m_l * m_size + i];
	cov[i*cov_size + i_c_l] = var_res * m[i * m_size + i_m_l];
      }

      cov[i_c_l*cov_size + i_c_a4] = var_res * m[i_m_l * m_size + i_m_a4];
      cov[i_c_a4*cov_size + i_c_l] = var_res * m[i_c_a4 * m_size + i_m_l];
    
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }

  case MDL_QUA_PAN_DIV: {// parameter c1, a1, q1, q2
    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_c_a1 = 2; // index of a1 parameter in covariance matrix
    i_c_q1 = 6; // index of q1 parameter in covariance matrix
    i_c_q2 = 7; // index of q2 parameter in covariance matrix

    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_m_a1 = 1; // index of a1 parameter in input matrix
    i_m_q1 = 2; // index of q1 parameter in input matrix
    i_m_q2 = 3; // index of q2 parameter in input matrix

    // Update the covariance matrix considering the motion model parameters
    cov[i_c_c1] = var_res * m[i_m_c1];
    cov[i_c_c1 * cov_size + i_c_a1] = var_res * m[i_m_c1 * m_size + i_m_a1];
    cov[i_c_c1 * cov_size + i_c_q1] = var_res * m[i_m_c1 * m_size + i_m_q1];
    cov[i_c_c1 * cov_size + i_c_q2] = var_res * m[i_m_c1 * m_size + i_m_q2];

    cov[i_c_a1 * cov_size + i_c_c1] = var_res * m[i_m_a1 * m_size + i_m_c1];
    cov[i_c_a1 * cov_size + i_c_a1] = var_res * m[i_m_a1 * m_size + i_m_a1];
    cov[i_c_a1 * cov_size + i_c_q1] = var_res * m[i_m_a1 * m_size + i_m_q1];
    cov[i_c_a1 * cov_size + i_c_q2] = var_res * m[i_m_a1 * m_size + i_m_q2];

    cov[i_c_q1 * cov_size + i_c_c1] = var_res * m[i_m_q1 * m_size + i_m_c1];
    cov[i_c_q1 * cov_size + i_c_a1] = var_res * m[i_m_q1 * m_size + i_m_a1];
    cov[i_c_q1 * cov_size + i_c_q1] = var_res * m[i_m_q1 * m_size + i_m_q1];
    cov[i_c_q1 * cov_size + i_c_q2] = var_res * m[i_m_q1 * m_size + i_m_q2];

    cov[i_c_q2 * cov_size + i_c_c1] = var_res * m[i_m_q2 * m_size + i_m_c1];
    cov[i_c_q2 * cov_size + i_c_a1] = var_res * m[i_m_q2 * m_size + i_m_a1];
    cov[i_c_q2 * cov_size + i_c_q1] = var_res * m[i_m_q2 * m_size + i_m_q1];
    cov[i_c_q2 * cov_size + i_c_q2] = var_res * m[i_m_q2 * m_size + i_m_q2];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      cov[i_c_l*cov_size + i_c_c1] = var_res * m[i_m_l * m_size + i_m_c1];
      cov[i_c_l*cov_size + i_c_a1] = var_res * m[i_m_l * m_size + i_m_a1];
      cov[i_c_l*cov_size + i_c_q1] = var_res * m[i_m_l * m_size + i_m_q1];
      cov[i_c_l*cov_size + i_c_q2] = var_res * m[i_m_l * m_size + i_m_q2];
      cov[i_c_c1*cov_size + i_c_l] = var_res * m[i_m_c1 * m_size + i_m_l];
      cov[i_c_a1*cov_size + i_c_l] = var_res * m[i_m_a1 * m_size + i_m_l];
      cov[i_c_q1*cov_size + i_c_l] = var_res * m[i_m_q1 * m_size + i_m_l];
      cov[i_c_q2*cov_size + i_c_l] = var_res * m[i_m_q2 * m_size + i_m_l];
     
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }
  case MDL_QUA_PAN_TILT: {// parameter c1, c2, q1, q2
    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_c_c2 = 1; // index of c2 parameter in covariance matrix
    i_m_c2 = 1; // index of c2 parameter in input matrix

    i_c_q1 = 6; // index of q1 parameter in covariance matrix
    i_m_q1 = 2; // index of q1 parameter in input matrix
    i_c_q2 = 7; // index of q2 parameter in covariance matrix
    i_m_q2 = 3; // index of q2 parameter in input matrix

    // Update the covariance matrix considering the motion model parameters
    cov[i_c_c1] = var_res * m[i_m_c1];
    cov[i_c_c1 * cov_size + i_c_c2] = var_res * m[i_m_c1 * m_size + i_m_c2];
    cov[i_c_c1 * cov_size + i_c_q1] = var_res * m[i_m_c1 * m_size + i_m_q1];
    cov[i_c_c1 * cov_size + i_c_q2] = var_res * m[i_m_c1 * m_size + i_m_q2];

    cov[i_c_c2 * cov_size + i_c_c1] = var_res * m[i_m_c2 * m_size + i_m_c1];
    cov[i_c_c2 * cov_size + i_c_c2] = var_res * m[i_m_c2 * m_size + i_m_c2];
    cov[i_c_c2 * cov_size + i_c_q1] = var_res * m[i_m_c2 * m_size + i_m_q1];
    cov[i_c_c2 * cov_size + i_c_q2] = var_res * m[i_m_c2 * m_size + i_m_q2];

    cov[i_c_q1 * cov_size + i_c_c1] = var_res * m[i_m_q1 * m_size + i_m_c1];
    cov[i_c_q1 * cov_size + i_c_c2] = var_res * m[i_m_q1 * m_size + i_m_c2];
    cov[i_c_q1 * cov_size + i_c_q1] = var_res * m[i_m_q1 * m_size + i_m_q1];
    cov[i_c_q1 * cov_size + i_c_q2] = var_res * m[i_m_q1 * m_size + i_m_q2];

    cov[i_c_q2 * cov_size + i_c_c1] = var_res * m[i_m_q2 * m_size + i_m_c1];
    cov[i_c_q2 * cov_size + i_c_c2] = var_res * m[i_m_q2 * m_size + i_m_c2];
    cov[i_c_q2 * cov_size + i_c_q1] = var_res * m[i_m_q2 * m_size + i_m_q1];
    cov[i_c_q2 * cov_size + i_c_q2] = var_res * m[i_m_q2 * m_size + i_m_q2];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      cov[i_c_l*cov_size + i_c_c1] = var_res * m[i_m_l * m_size + i_m_c1];
      cov[i_c_l*cov_size + i_c_c2] = var_res * m[i_m_l * m_size + i_m_c2];
      cov[i_c_l*cov_size + i_c_q1] = var_res * m[i_m_l * m_size + i_m_q1];
      cov[i_c_l*cov_size + i_c_q2] = var_res * m[i_m_l * m_size + i_m_q2];
      cov[i_c_c1*cov_size + i_c_l] = var_res * m[i_m_c1 * m_size + i_m_l];
      cov[i_c_c2*cov_size + i_c_l] = var_res * m[i_m_c2 * m_size + i_m_l];
      cov[i_c_q1*cov_size + i_c_l] = var_res * m[i_m_q1 * m_size + i_m_l];
      cov[i_c_q2*cov_size + i_c_l] = var_res * m[i_m_q2 * m_size + i_m_l];
     
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }
  case MDL_QUA_PAN_TILT_DIV: {// parameter c1, c2, a1, q1, q2
    i_c_c1 = 0; // index of c1 parameter in covariance matrix
    i_m_c1 = 0; // index of c1 parameter in input matrix
    i_c_c2 = 1; // index of c2 parameter in covariance matrix
    i_m_c2 = 1; // index of c2 parameter in input matrix
    i_c_a1 = 2; // index of c2 parameter in covariance matrix
    i_m_a1 = 2; // index of c2 parameter in input matrix

    i_c_q1 = 6; // index of q1 parameter in covariance matrix
    i_m_q1 = 3; // index of q1 parameter in input matrix
    i_c_q2 = 7; // index of q2 parameter in covariance matrix
    i_m_q2 = 4; // index of q2 parameter in input matrix

    // Update the covariance matrix considering the motion model parameters
    cov[i_c_c1] = var_res * m[i_m_c1];
    cov[i_c_c1 * cov_size + i_c_c2] = var_res * m[i_m_c1 * m_size + i_m_c2];
    cov[i_c_c1 * cov_size + i_c_a1] = var_res * m[i_m_c1 * m_size + i_m_a1];
    cov[i_c_c1 * cov_size + i_c_q1] = var_res * m[i_m_c1 * m_size + i_m_q1];
    cov[i_c_c1 * cov_size + i_c_q2] = var_res * m[i_m_c1 * m_size + i_m_q2];

    cov[i_c_c2 * cov_size + i_c_c1] = var_res * m[i_m_c2 * m_size + i_m_c1];
    cov[i_c_c2 * cov_size + i_c_c2] = var_res * m[i_m_c2 * m_size + i_m_c2];
    cov[i_c_c2 * cov_size + i_c_a1] = var_res * m[i_m_c2 * m_size + i_m_a1];
    cov[i_c_c2 * cov_size + i_c_q1] = var_res * m[i_m_c2 * m_size + i_m_q1];
    cov[i_c_c2 * cov_size + i_c_q2] = var_res * m[i_m_c2 * m_size + i_m_q2];

    cov[i_c_q1 * cov_size + i_c_c1] = var_res * m[i_m_q1 * m_size + i_m_c1];
    cov[i_c_q1 * cov_size + i_c_c2] = var_res * m[i_m_q1 * m_size + i_m_c2];
    cov[i_c_q1 * cov_size + i_c_a1] = var_res * m[i_m_q1 * m_size + i_m_a1];
    cov[i_c_q1 * cov_size + i_c_q1] = var_res * m[i_m_q1 * m_size + i_m_q1];
    cov[i_c_q1 * cov_size + i_c_q2] = var_res * m[i_m_q1 * m_size + i_m_q2];

    cov[i_c_q2 * cov_size + i_c_c1] = var_res * m[i_m_q2 * m_size + i_m_c1];
    cov[i_c_q2 * cov_size + i_c_c2] = var_res * m[i_m_q2 * m_size + i_m_c2];
    cov[i_c_q2 * cov_size + i_c_a1] = var_res * m[i_m_q2 * m_size + i_m_a1];
    cov[i_c_q2 * cov_size + i_c_q1] = var_res * m[i_m_q2 * m_size + i_m_q1];
    cov[i_c_q2 * cov_size + i_c_q2] = var_res * m[i_m_q2 * m_size + i_m_q2];

    // Update the covariance matrix considering the var light parameter
    if (param->var_light) {
      i_c_l  = cov_size - 1; // index of light parameter in covariance matrix
      i_m_l  = nb_para;      // index of light parameter in covariance matrix
      cov[i_c_l*cov_size + i_c_c1] = var_res * m[i_m_l * m_size + i_m_c1];
      cov[i_c_l*cov_size + i_c_c2] = var_res * m[i_m_l * m_size + i_m_c2];
      cov[i_c_l*cov_size + i_c_a1] = var_res * m[i_m_l * m_size + i_m_a1];
      cov[i_c_l*cov_size + i_c_q1] = var_res * m[i_m_l * m_size + i_m_q1];
      cov[i_c_l*cov_size + i_c_q2] = var_res * m[i_m_l * m_size + i_m_q2];
      cov[i_c_c1*cov_size + i_c_l] = var_res * m[i_m_c1 * m_size + i_m_l];
      cov[i_c_c2*cov_size + i_c_l] = var_res * m[i_m_c2 * m_size + i_m_l];
      cov[i_c_a1*cov_size + i_c_l] = var_res * m[i_m_a1 * m_size + i_m_l];
      cov[i_c_q1*cov_size + i_c_l] = var_res * m[i_m_q1 * m_size + i_m_l];
      cov[i_c_q2*cov_size + i_c_l] = var_res * m[i_m_q2 * m_size + i_m_l];
     
      cov[i_c_l * cov_size + i_c_l] = var_res * m[i_m_l * m_size + i_m_l];
    }
    break;
  }
  
  } // end switch()
  if (DEBUG_LEVEL2)
    print_covariance(param);

  return true;
}


/*
  INPUT       :
  covariance   The covariance matrix.
 
  DESCRIPTION :
  Initialize the covariance matrix to infinity.
 
*/

void init_covariance(Para *param)
{
  //double   infinity = 100000000.0;
  double   infinity = 1000000.0;

  for (int i=0; i < MAXCOEFSMODEL*MAXCOEFSMODEL; i ++) {
    param->covariance[i] = infinity;
  }
}

/*

  Print the content of the covariance matrix

*/
void print_covariance(Para *param)
{
  int      cov_size = MAXCOEFSMODEL;
  printf("Covariance matrix - model=%d nb_para=%d:\n",
	 param->id_model, param->nb_para);
  for(int i = 0; i < cov_size; i ++) {
    for(int j = 0; j < cov_size; j ++) {
      printf("%g ", param->covariance[i*cov_size + j]);
    }
    printf("\n");
  }
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
