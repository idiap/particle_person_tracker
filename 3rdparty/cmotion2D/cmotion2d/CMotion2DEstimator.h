/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

/*!
  \file CMotion2DEstimator.h
  \brief File to include to use CMotion2DEstimator.
*/

#ifndef CMotion2DEstimator_h
#define CMotion2DEstimator_h

#include <stdio.h>

#include "CMotion2DPyramid.h"
#include "CMotion2DModel.h"


#if defined (WIN32)
#  if defined MOTION2D_DLL_EXPORTS
#     define MOTION2D_API __declspec( dllexport )
#  elif defined MOTION2D_DLL_IMPORTS
#     define MOTION2D_API __declspec( dllimport )
#  else
#     define MOTION2D_API
#  endif
#else
#     define MOTION2D_API
#endif



class MOTION2D_API CMotion2DEstimator
{
 public:

  /*!

    \enum ERobustFunction Defines implemented robust functions \f$\rho(x)\f$
    that can be used to compute the weights \f$w_i\f$.

    \sa setRobustFunction(), ECConstType

  */
  enum ERobustFunction {
    Tukey,
    /*!< Tukey biweight function: if \f$\mid r_i \mid < C \makebox[0.3cm]{} w_i
      = (1-(\frac{r_i}{C})^2)^2\f$ otherwise \f$w_i=0\f$ */
    Talwar,
    /*!< Talwar biweight function: if \f$(\frac{r_i}{C})^2 \leq 1
      \makebox[0.3cm]{} w_i = 0 \f$ otherwise \f$w_i=1\f$ */
    Cauchy,
    /*!< Cauchy function: \f$ w_i = \frac{1}{1+(\frac{r_i}{C})^2} \f$*/
    Welsh
    /*!< Welsh function: \f$w_i = exp( - (\frac{r_i}{C})^2) \f$*/
  };

  /*!

    \enum ECConstType Describes the way in which the \f$C\f$ constant evolves
    between two successive estimations.

    At the beginning of the process, the estimator must be rather tolerant to
    outliers; thus, \f$C\f$ is set to a large value (typically equal to the
    largest absolute temporal intensity difference). Then, as the accuracy of
    the dominant motion estimate improves, outliers are better identified and
    should be rejected. Therefore, \f$C\f$ is lowered at each computation of a
    new increment \f$\Delta \Theta_k\f$, using the following schedule: \f$C_k =
    0.9 C_{k-1}\f$ until it reaches a preset value \f$C_p\f$, or a robustly
    estimated value \f$C_r\f$.

    \sa setCConstant(), ERobustFunction */
  enum ECConstType {
    CConst_Fixed,
    /*!< The \f$C\f$ constant does not change between two estimations of
    \f$\Delta \Theta_k\f$.*/
    CConst_Classic,
    /*!< The \f$C\f$ constant is lowered at each computation of a new increment
    according to the formula \f$C_k = 0.9 C_{k-1}\f$. */
    CConst_Robust
    /*!< The \f$C\f$ constant is robustly estimated.  */
  };

 public:
  CMotion2DEstimator();
  ~CMotion2DEstimator();

  bool estimate(const CMotion2DPyramid & pyramid1,
		const CMotion2DPyramid & pyramid2,
		const short *support, short label,
		CMotion2DModel &model, bool useModelAsInitialization=false);
  bool estimate(const CMotion2DPyramid & pyramid1,
		const CMotion2DPyramid & pyramid2,
		const unsigned char *support, unsigned char label,
		CMotion2DModel &model, bool useModelAsInitialization=false);
  float *getWeights(int &n_rows, int &n_cols);


  /*!

    If \e robust is true, a robust estimator is used. Otherwise, a
    least-mean-square method is considered, where all the weights are set to 1.

    \sa setRobustFunction(), setNIterationMaxIRLS()

  */
  void setRobustEstimator(bool robust=true) {
    if (robust == true)
      least_mean_square = false; // iterated weighted least-square
    else {
      least_mean_square = true; // just least-square.
      //setNIterationMaxIRLS(1);
    }
  };

  /*!

    Defines the robust function used in the weighted least-squares resolution
    (IRLS). ERobustFunction() gives the list of implemented functions.

    \sa ERobustFunction(), setRobustEstimator(), setNIterationMaxIRLS(),
    setCConstant()

  */
  void setRobustFunction(ERobustFunction function=Tukey) {
    switch (function) {
    case Tukey : ty_pond = 0; break;
    case Talwar: ty_pond = 1; break;
    case Cauchy: ty_pond = 2; break;
    case Welsh : ty_pond = 3; break;
    }
  };
  /*!

    Sets the \f$C_p\f$ constant used to reject outliers when a robust
    M-estimator is used.

    \sa ECConstType(), setNIterationMaxIRLS(), setRobustFunction()

  */
  void setCConstant(ECConstType C_type=CConst_Fixed, double C=8.0) {
    switch (C_type) {
    case CConst_Fixed:
      type_variance = 0; // Fixed C constant.
      variance =  C; // Final value of the C constant
      break;
    case CConst_Classic:
      type_variance = 1;
      break;
    case CConst_Robust:
      type_variance = 2;
      break;
    }
  };

  /*!

    Sets the maximal number of iterations in the iteratively reweighted least
    squares (IRLS) method. IRLS first consists in evaluating the residual
    \f$r_i\f$, and, consequently the weights \f$w_i\f$. Then, a new estimate of
    \f$\Delta\Theta\f$ is computed using the weighted least-squares
    technique. The weights are updated, and the process is repeated until
    convergence, or until the predefined number of iterations \e niter is
    reached.

    \sa setRobustEstimator(), setRobustFunction(), setCConstant()

  */
  void setNIterationMaxIRLS(int niter=8) {
    max_it_irls = niter;
  };

  /*!

    Sets the maximal number of iterations of IRLS performed at a given pyramid
    level \e l to estimate \f$ \widehat{\Delta \Theta^l}\f$.

    \sa setNIterationMaxIRLS()

  */
  void setNIterationMaxLevel(int niter=7) {
    max_it_stab = niter;
  };

  /*!

    Sets the different pyramid levels where respectively the constant, affine
    and quadratic parameters of the 2D motion model will be first introduced
    and estimated.

    If a parameter is set to -1, the default level is used.

  */
  void setLevelParameterIntroduction(int level_const=-1,
				     int level_affine=-1,
				     int level_quadratic=-1) {

    level_ct   = level_const;
    level_lin  = level_affine;
    level_quad = level_quadratic;
  };

  /*!

    Defines the pyramid first level where the estimation process is starting,
    with: \f$ 0 \leq level < NLEVELS\_MAX \f$

    At the coarsest level \e l, no prior estimation is available. The weights
    are set to 1 and the mimimization is started using expression <a
    href=classCMotion2DEstimator.html#equation1">(1)</a>. A first estimate of
    \f$\Theta^l\f$ is then obtained and successive refinements are performed at
    the same level considering expression <a
    href=classCMotion2DEstimator.html#equation3">(3)</a> until the incremental
    estimate \f$\widehat{\Delta \Theta^l}\f$ is small enough or a given number
    of iterations fixed by set setNIterationMaxIRLS() is reached. Then, the
    estimated parameter \f$\widehat \Theta^l \f$ is projected to the finer
    level, where the refinement process starts again by minimizing expression
    <a href=classCMotion2DEstimator.html#equation3">(3)</a>.

    Returns false if the requested level is not authorized, true otherwise.

    \sa getFirstEstimationLevel(), setLastEstimationLevel(),
    getLastEstimationLevel()

  */
  bool setFirstEstimationLevel(unsigned level=3) {
    if (level >= (unsigned) CMotion2DPyramid::NLEVELS_MAX) {
       __panic((char*)"Cmotion2DEstimator::setFirstEstimationLevel() : Bad level...");
       return false;
    }
    first_est_level = level;
    return true;
  };

  /*!

    Returns the pyramid level where the estimation process is
    started. Considerering the pyramids size, this level can be different from
    the level specified with setFirstEstimationLevel(). The returned value is
    updated after each call of the estimate() method.

    \sa setFirstEstimationLevel(), setLastEstimationLevel(),
    getLastEstimationLevel()

  */
  unsigned getFirstEstimationLevel() {
    return pyr_level_max;
  };

  /*!

    Sets the pyramid level where the estimation process is stopped, with \f$ 0
    \leq level < NLEVELS\_MAX \f$

    Returns false if the requested level is not authorized, true otherwise.

    \sa setFirstEstimationLevel(), getFirstEstimationLevel(),
    getLastEstimationLevel()

  */
  bool setLastEstimationLevel(unsigned level=0) {
    if (level >= (unsigned) CMotion2DPyramid::NLEVELS_MAX) {
       __panic((char*)"Cmotion2DEstimator::setFirstEstimationLevel() : Bad level...");
       return false;
    }

    final_est_level = level;
    return true;
  };

  /*!

    Returns the pyramid level where the estimation process is stopped.

    \sa setLastEstimationLevel(), setFirstEstimationLevel(),
    getFirstEstimationLevel()


  */
  unsigned getLastEstimationLevel() {
    return final_est_level;
  };

  /*!

    Sets the support percentage (\e rate) for which the estimation is
    considered as reliable. It must be comprised between 0.f and 1.f.

  */
  void setReliableSupportRate(float rate=0.4) {
      if ((rate < 0.f) || (rate > 1.f)) {
	__panic((char*)"CMotion2DEstimator::setReliableSupportRate(): Bad rate...");
      }
      tx_pts_min = rate;
  };

  /*!

    If \e compute is set as true, the normalized support size evaluated from
    the weights \f$ w_i \f$ is computed. If the weight associated to a given
    pixel is greater than \e threshold, this pixel is considered as conform to
    the dominant motion. The support size corresponds to the ratio between the
    number of pixels conform to the estimated dominant motion and the size of
    the part of \f$I_t\f$ that is likely to overlap \f$I_{t+1}\f$.

    The \e threshold value must belong to [0; 1[.

    \sa getSupportSize(), isSupportSizeComputed(), getWeights()

  */
  void computeSupportSize(bool compute=true, float threshold = 0.2f) {
    if (compute == true) {
      compute_support_size = 1;	// Calcul nb pixels conformes par rapport au
				// nb pixels zone de recouvrement.
      if ((threshold < 0.f) || (threshold > 1.f)) {
	__panic((char*)"CMotion2DEstimator::computeSupportSize(): Bad threshold.");
      }
      seuil_poids = threshold;	// Seuil au dela duquel 1 pixel est
      // conforme au mvt.
    }
    else
      compute_support_size = 0;
  };

  /*!

    Indicates if the normalized support size is computed.

    \sa computeSupportSize()

  */
  bool isSupportSizeComputed() {
    if (compute_support_size == 1)
      return true;
    else
      return false;
  };

  /*!

    Returns the normalized size of the estimated weight map. If the method
    returns true, the normalized support size is computed.

    \sa computeSupportSize(), getWeights()

  */
  bool getSupportSize(double &size) {
    if (compute_support_size == 1) {
      size = rapport_poids;
      return true;
    }
    else {
      // The support size is not computed
      size = 0.0;
      return false;
    }
  };

  /*!

    If \e compute is set as true, the residual variance of the estimated 2D
    polynomial motion model parameters is computed.

    \sa isResidualVarianceComputed()

  */
  void computeResidualVariance(bool compute=false) {
    compute_sigma2res = compute;
  };

  /*!

    Indicates if the variance of the estimated 2D motion model parameters is
    computed. In fact, it is the variance of the last estimated parameter
    increment.

    \sa computeResidualVariance()

  */
  bool isResidualVarianceComputed() {
    if (compute_sigma2res == true)
      return true;
    else
      return false;
  };

  /*!

    Returns the variance of the residual values.

    \sa computeResidualVariance()

   */
  double getResidualVariance() {return sigma2res; };

  /*!

    If \e compute is true, computes the covariance matrix of the estimated 2D
    polynomial motion model parameters.

    \sa isCovarianceMatrixComputed(), getCovarianceMatrix()

  */
  void computeCovarianceMatrix(bool compute=false) {
    compute_covariance = compute;
  };

  /*!

    Indicates if the covariance matrix of the estimated 2D motion model
    parameters is computed.

    \sa computeCovarianceMatrix(), getCovarianceMatrix()

  */
  bool isCovarianceMatrixComputed() {
    if (compute_covariance == true)
      return true;
    else
      return false;
  };
  /*!

    Updates the address of the covariance matrix containing the covariance
    terms of the estimated parameters. The covariance matrix is bidimentional,
    with MAXCOEFSMODEL rows and MAXCOEFSMODEL colomns. If the matrix is not
    computed, the method returns NULL.

    \return true if the covariance matrix is computed, false otherwise.

    \sa computeCovarianceMatrix(), isCovarianceMatrixComputed()

  */
  double *getCovarianceMatrix(int &n_rows, int &n_cols){
    if (compute_covariance == true) {
      n_rows = MAXCOEFSMODEL;
      n_cols = MAXCOEFSMODEL;
      return covariance;
    }
    else {
      // The covariance matrix is not computed
      n_rows = -1;
      n_cols = -1;
      return NULL;
    }
  };

  float *getDisplacedRowsSpatialGradientDataAddress(int level, int &n_rows, int &n_cols);
  float *getDisplacedColsSpatialGradientDataAddress(int level, int &n_rows, int &n_cols);
  float *getDFDDataAddress(int level, int &n_rows, int &n_cols);


 private:
  void __panic(char *message);

  ///////////////////////////////////////////////
  // For the estimator tuning.
  ///////////////////////////////////////////////
  int nrows, ncols;
  int type_variance;	// Variance given (0), classic (1), robust (2).
  int ty_pond;		// Weights computation: Tukey(0), Talwar(1),Cauchy(2),
			// Welsch (3).
  int max_it_stab;	// Max number of iterations at a given pyramid level
  int max_it_irls;	// Max number of iterations in the IRLS method.
  double variance;	// Final value of the C constant
  int level_ct;		// Input level where constant models are considered.
  int level_lin;	// Input level where linear models are considered.
  int level_quad;	// Input level where quadratic models are considered.
  unsigned first_est_level;// Pyramid level where the estimation starts.
  unsigned final_est_level;// Pyramid level where the estimation stops
  int pyr_level_max; // First pyramids level used for motion estimation
  bool compute_sigma2res; //  If true compute the motion model variance.
  bool compute_covariance; //  If true compute the covariance matrix
  bool verbose;		// Verbose mode.

  double seuil_poids;	// Threshold to determine if a pixel is conform
  bool least_mean_square;// If false robust estimation
  double tx_pts_min;	// Min points requiered at a level

  bool init_mem_estIsDone;		// If 1, estimator memory allocated.
  bool init_mem_weightsIsDone;		// If 1, memory allocated for weighs
  TImageFloat pyr_fl1[CMotion2DPyramid::NLEVELS_MAX];	// Displaced gradients along x.
  TImageFloat pyr_fl2[CMotion2DPyramid::NLEVELS_MAX];	// Displaced gradients along y.
  TImageFloat pyr_fl3[CMotion2DPyramid::NLEVELS_MAX];	// DFD: I(pi+depl,t+1)-I(pi,t).
  TImageFloat pyr_weights[CMotion2DPyramid::NLEVELS_MAX];// Weights pyramid
  TImageShort pyr_support[CMotion2DPyramid::NLEVELS_MAX];// Support pyramid
  int ct_level_intro;	// Level where constant models were considered
  int lin_level_intro;	// Level where linear models were considered.
  int quad_level_intro;	// Level where quadratic models were considered.
  int compute_support_size; // If true, compute the normalized ratio between
			// pixels considered as conform to the dominant
			// motion and the size of of the part of I(t) that is
			// likely to overlap I(t+1).
  double rapport_poids;	// Normalized ratio (see compute_support_size)
  double sigma2res;	// Residual variance.
  double covariance[MAXCOEFSMODEL * MAXCOEFSMODEL];	// Covariance matrix

};


#endif
