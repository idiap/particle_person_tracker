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
  \file CMotion2DEstimator.cpp
  \brief Definition of the CMotion2DEstimator class.
*/

/*!

  \class CMotion2DEstimator

  \brief The CMotion2DEstimator class implements the robust multiresolution
  estimation of the parametric motion model.

  We consider the displaced frame difference corresponding to the motion model:

  \f[
  DFD_{\Theta}(p_i) = I_{t+1}(p_i + \vec{w}_{A}(p_{i})) - I_t(p_i) + \zeta
  \f]

  where \f$I_t\f$ and \f$I_{t+1}\f$ are two successive images, \f$\Theta^t =
  (A^t, \zeta)\f$ the motion model to estimate (including a global intensity
  shift \f$\zeta\f$ to account for global illumination change) and

  \f[
  \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
  \f]

  denotes the flow vector modeled at the image point \f$ p_i\f$.

  To ensure the goal of robustness, we introduce a M-estimation criterion with
  a hard-redescending robust estimator. Thus, the motion model is given by:

  <a name=equation1>

  \f[ \widehat{\Theta} = \mbox{arg}\!\min_{\Theta}\,\,
  \sum_{p_i \in R_t} \rho\left(\mbox{DFD}_{\Theta}(p_i), C \right)
  \mbox{\hspace{1cm}} (1) \f]

  where \f$\rho(x)\f$ is the Tukey, Talwar, Cauchy or Welsh biweight function
  which is bounded for high values of \f$x\f$ and \f$C\f$ a scale parameter to
  be set.

  The estimation support \f$R_t\f$ usually consists of the whole image. If
  required, it can also be restricted to a specific area of the image.  The
  minimization is embedded in a multiresolution and incremental scheme based on
  the Gauss-Newton method.  At each incremental step \f$k\f$ (at a given
  resolution level, or from a resolution level to a finer one), we have:

  <a name=equation2>

  \f[ \Theta=\widehat\Theta_k+\Delta \Theta_k \makebox[1.5cm]{with} \left\{
  \begin{array}{l} A = \widehat{A_k} + \Delta A_k \\ \zeta = \widehat{\zeta_k}
  + \Delta \zeta \end{array} \right.   \mbox{\hspace{1cm}} (2) \f]

  where \f$\widehat\Theta_k\f$ is the current estimate of the parameter vector
  \f$\Theta\f$.  A linearization of \f$\mbox{DFD}_{\Theta}(p_i)\f$ around
  \f$\widehat\Theta_k\f$ is performed, leading to the expression \f$r_{\Delta
  \Theta_k}(p_i)\f$ linear with respect to \f$\Delta \Theta_k\f$:

  \f[ r_{\Delta \Theta_k}(p_i) = \vec{\nabla} I_{t+1}(p_i+
   \vec{w}_{\widehat{A_k}}(p_i)).\vec{w}_{\Delta A_k}(p_i) + \Delta \zeta_k +
   I_{t+1}(p_i+ \vec{w}_{\widehat{A_k}}(p_i))-I_t(p_i) + \widehat{\zeta_k} \f]

  where \f$\vec\nabla I_{t+1}(p_i)\f$ denotes the spatial gradient of the
  intensity function at location \f$p_i\f$ and at time \f$t+1\f$.  Then, we
  substitute for the minimization of \f$E(\Theta)\f$ the minimization
  of the expression given by:

  \f[E(\Delta\Theta_k)= \sum_{p_i \in R_t} \rho(r_{\Delta \Theta_k}(p_i),
  C) \f]

  which is equivalent to minimizing:

  <a name=equation3>

  \f[\sum_{p_i \in R_t} w_i . r_{\Delta \Theta_k}^2 (p_i) \mbox{\hspace{1cm}}
  (3) \f]

  with

  <a name=equation4>

  \f[w_i = \frac{\psi(r_{\Delta \Theta_k}(p_i))}{r_{\Delta \Theta_k}(p_i))}
  \mbox{\hspace{1cm}} (4) \f]

  where \f$\psi\f$ is the derivative of the \f$\rho\f$ function.  The error
  function \f$E(\Delta\Theta_k)\f$ is minimized using an
  Iterative-Reweighted-Least-Squares procedure, with 0 as an initial value for
  \f$\Delta \Theta_k\f$.  For more details about the method and its
  performances, see the Motion-2D related references.

*/

#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif

#include <stdlib.h>

#include "type.h"
#include "constant.h"
#include "macro.h"

#include "../cprim/acast.h"
#include "../pyramide/filt_gauss.h"
#include "../estimation/RMRmod.h"
#include "../estimation/cog.h"
#include "../memoire/memoire.h"
#include "../estimation/para_mvt.h"
#include "../estimation/mem_est.h"
#include "../estimation/covariance.h"


#include <cmotion2d/CMotion2DPyramid.h>
#include <cmotion2d/CMotion2DEstimator.h>


#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0

/*!

  Constructs a robust multiresolution estimator by implementing
  the iterative reweighted least squares (IRLS) technique.

*/
CMotion2DEstimator::CMotion2DEstimator()
{
  if (DEBUG_LEVEL1)
    cout << "Debut CMotion2DEstimator::CMotion2DEstimator()" << endl;
  rapport_poids = 0.0;
  sigma2res     = 0.0;
  verbose	= false;

  setRobustEstimator(true);
  setNIterationMaxLevel(7);

  setLastEstimationLevel(0);
  setFirstEstimationLevel(3);
  setLevelParameterIntroduction(-1, -1, -1);

  setRobustFunction(Tukey);
  setCConstant(CConst_Fixed, 8.0);
  setNIterationMaxIRLS(8);
  setReliableSupportRate(0.4f);

  computeSupportSize(true, 0.2f);
  computeResidualVariance(false);
  computeCovarianceMatrix(false);
  nrows = 0;
  ncols = 0;
  pyr_level_max = 0;

  init_mem_weightsIsDone = false;
  init_mem_estIsDone = false;

  if (DEBUG_LEVEL1)
    cout << "Fin CMotion2DEstimator::CMotion2DEstimator()" << endl;
}

/*!

  Destructs the multiresolution estimator, deleting all allocated memory.

*/
CMotion2DEstimator::~CMotion2DEstimator()
{
  if (DEBUG_LEVEL1)
    cout << "Debut CMotion2DEstimator::~CMotion2DEstimator()" << endl;

  if (init_mem_weightsIsDone == true) {
    if (DEBUG_LEVEL2)
      cout << "free pyr_weights[] nlevels: " << pyr_level_max << endl;
    free_p_fl(&pyr_weights[0], pyr_level_max + 1);
    init_mem_weightsIsDone = false;
  }

  // pyramides specifiques a l'estimation.
  if (init_mem_estIsDone == true) {
    if (DEBUG_LEVEL2) cout << "free pyr_est[]" << endl;

    efface_memoire_pyr_est(pyr_level_max+1, &init_mem_estIsDone,
			   &pyr_fl1[0], &pyr_fl2[0],
			   &pyr_fl3[0], &pyr_support[0]);
    init_mem_estIsDone = false;
  }


  // Coefficients du filtre gaussien.
  efface_memoire_filtre_gauss();
  if (DEBUG_LEVEL1)
    cout << "Fin CMotion2DEstimator::~CMotion2DEstimator()" << endl;
}


/*!

  Implements the estimation of a 2D parametric motion model.

  Depending on setRobustEstimator() two different schemes can be used: either
  involving a robust multiresolution estimation implementing the IRLS
  technique, or involving a multiresolution least-mean-square estimation.

  The estimation process is performed between images \f$I_t\f$ and
  \f$I_{t+1}\f$ for which the Gaussian and image gradient pyramids, denoted
  respectively \e pyramid1 and \e pyramid2, must be built using
  CMotion2DPyramid.

  The parameter \e support refers to the estimation support \f$R_t\f$. In other
  terms, \e support indicates which pixels of the image \f$I_t\f$ are taken
  into account in the estimation process. The size of \e support must be equal
  to the size of the finest level in the pyramids (\e pyramid1 or \e
  pyramid2). It usually consists of the whole image. In that case, all \e
  support must be set to \e label. But, if required, it can also be restricted
  to a specific area of the image. In that case, all the pixels belonging to
  this area must be set to \e label. The other pixels must be set to another
  value than \e label.

  An initialization of the motion \e model parameters does not affect the
  results.  That means that the parameters of the motion model given as input
  are not used to initialize the estimator.

  This function returns the estimated 2D parametric motion model in \e
  model. The returned parameters have to be considered for the highest
  resolution in the pyramids (level = 0) even if the last level considered in
  the estimation process is different when specified for example with
  setLastEstimationLevel(1). If the estimation process is stopped at a pyramid
  level different from 0, a projection of the motion model parameter to level 0
  is performed. For the projection from one given level to the next finer one,
  the constant parameters of the model are multiplied by 2, the affine
  parameters are unchanged and the quadratic parameters are divided by 2.

  The weight map can be obtained by getWeights().

  Returns true if the estimation process succeeds, false otherwise.

  \sa estimate(const CMotion2DPyramid &, const CMotion2DPyramid &, const unsigned char *, unsigned char, CMotion2DModel &), setRobustEstimator(), setFirstEstimationLevel(),
  setLastEstimationLevel(), getWeights(),


*/
bool CMotion2DEstimator::estimate(const CMotion2DPyramid & pyramid1,
				  const CMotion2DPyramid & pyramid2,
				  const short *support, short label,
				  CMotion2DModel & model,
				  bool useModelAsInitialization)
{
  if (DEBUG_LEVEL1)
    cout << "Begin CMotion2DEstimator::estimate()" << endl;

  Para paramet;
  float ligravbase = 0.;  /* Coordonnees du centre de gravite des regions.*/
  float cogravbase = 0.;  /* Coordonnees du centre de gravite des regions.*/
  int i;
  int Mode_Niv_Init;	  /* estime para cst, lin... avec niveaux par defaut */
  int para_init = 0;	  /* 108: au depart on n'a pas d'estimee des parametres */
			  /* //� changer !! bug */

  bool state = false;

  // Check the pyramid size, if differ, return false
  int nrows_pyr1, ncols_pyr1;
  int nrows_pyr2, ncols_pyr2;
  if (pyramid1.getNumberOfRows(nrows_pyr1) != PYR_NO_ERROR)
    return false;
  if (pyramid1.getNumberOfCols(ncols_pyr1) != PYR_NO_ERROR)
    return false;
  if (pyramid2.getNumberOfRows(nrows_pyr2) != PYR_NO_ERROR)
    return false;
  if (pyramid2.getNumberOfCols(ncols_pyr2) != PYR_NO_ERROR)
    return false;
  if ((nrows_pyr1 != nrows_pyr2) || (ncols_pyr1 != ncols_pyr2)) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DEstimator::estimate() \n\tError: Pyramid size differ");
    __panic(message);
    return false;
  }

  if (DEBUG_LEVEL2) {
    cout << "Pyr 1: " << nrows_pyr1 << " x " << ncols_pyr1 << endl;
    cout << "Pyr 2: " << nrows_pyr2 << " x " << ncols_pyr2 << endl;
  }

  // Check if the memory initialization was done for the weights pyramid
  if (init_mem_weightsIsDone == true) {
    // Destruction of the weights pyramid if size differ
    if ((nrows != nrows_pyr1) || (ncols != ncols_pyr1)) {
      if (DEBUG_LEVEL3)
	cout << "free pyr_weights[] nlevels: " << pyr_level_max << endl;
      free_p_fl(&pyr_weights[0], pyr_level_max + 1);
      init_mem_weightsIsDone = false;
    }
  }

  // Check if the memory initialization was done for the internal pyramids
  if (init_mem_estIsDone == true) {
    // Destruction of the internal pyramids if size differ
    if ((nrows != nrows_pyr1) || (ncols != ncols_pyr1)) {
      if (DEBUG_LEVEL3)
	cout << "free mem_est nlevels: " << pyr_level_max << endl;
      efface_memoire_pyr_est(pyr_level_max + 1, &init_mem_estIsDone,
			     pyr_fl1, pyr_fl2, pyr_fl3, pyr_support);
      init_mem_estIsDone = false;
    }
  }

  // Initialisation:
  rapport_poids = 0.0;
  sigma2res     = 0.0;

  // Update the image size
  nrows         = nrows_pyr1;
  ncols         = ncols_pyr1;

  // Warning: To compute the covariance matrix we have to compute the residual
  // variance. Thus, if compute_covariance is true, we set compute_sigma2res to
  // true.
  if (compute_covariance)
    compute_sigma2res = true;

  // Initialize the covariance matrix to infinity
  init_covariance(&paramet);

  paramet.compute_covariance = compute_covariance;
  paramet.compute_sigma2res  = compute_sigma2res;
  paramet.sigma2res = 0.0;

  /* on prend les valeurs par d�faut */
  if( level_ct==-1 && level_lin==-1 && level_quad==-1 )
    Mode_Niv_Init=0;
  else {
    Mode_Niv_Init=1;
    if (level_quad >= level_lin) {
      char message[FILENAME_MAX];
      sprintf(message, "CMotion2DEstimator::estimate() \n\tBad level number %d for starting the estimation of quadratic parameters while affine parameters are estimate at level %d...", level_quad, level_lin);
      __panic(message);
      return false;
    }
    if (level_lin >= level_ct ) {
      char message[FILENAME_MAX];
      sprintf(message, "CMotion2DEstimator::estimate() \n\tBad level number %d for starting the estimation of linear parameters while constant parameters are estimate at level %d...", level_lin, level_ct);
      __panic(message);
      return false;
    }
  }

  if (first_est_level < final_est_level) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DEstimator::estimate() \n\tBad initial and final estimation levels...");
    __panic(message);
    return false;
  }

  // Modifcation of the first level
  int pyr1_level_max = pyramid1.getNumberOfLevels() - 1;
  int pyr2_level_max = pyramid2.getNumberOfLevels() - 1;
  pyr_level_max      = Min(pyr1_level_max, pyr2_level_max);

  if (pyr_level_max < 0) {
    // The pyramid was not build
    return false;
  }

  // Modification of the first estimation level, if this level is not
  // built in the pyramids
  if (pyr_level_max > (int) first_est_level)
    pyr_level_max = first_est_level;

  // Allocate memory for the weights pyramid
  if (init_mem_weightsIsDone == false) {
    if (DEBUG_LEVEL3) {
      cout << "alloc pyr_weights[] nlevels: " << pyr_level_max << endl;
      cout << "nrows: " << nrows << " ncols: " << ncols << endl;
    }
    if (Mem_pyramide_float(&pyr_weights[0], nrows, ncols, pyr_level_max)
	== false)
      return false;
    init_mem_weightsIsDone = true;
  }

  // Initialize the memory used by the estimator for "pyr_fl1", "pyr_fl2" which
  // contains the displaced spatial gradients, for "pyr_fl3" which contains the
  // DFD ie I(pi+depl,t+1)-I(pi,t), and for "pyr_support" which contains the
  // estimator support.
  if (init_mem_est(nrows, ncols, pyr_level_max, &init_mem_estIsDone,
		   pyr_fl1, pyr_fl2, pyr_fl3, pyr_support) == false) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DEstimator::estimate() \n\tCan not allocate memory...");
    __panic(message);
    return false;
  }

  // Initialize the estimation support
  memcpy(pyr_support[0].ad, support, nrows * ncols*sizeof (short));

  paramet.n_points = 0;

  paramet.nb_para = Nb_Para_Modele(model.getIdModel());

  // Test if the motion model exists
  if (paramet.nb_para==0)
  {
    __panic((char*)"CMotion2DEstimator::estimate() \n\tMotion model not implemented.\n");
    return false;
  }

  paramet.var_light = model.getVarLight();
  paramet.id_model  = model.getIdModel();

  if (useModelAsInitialization == true) {
    para_init = 1;
    model.getParameters(paramet.thet); // set the MDL_NMAX_COEF parameters
    model.getVarLight(paramet.thet[12]);
  }
  else {
    para_init = 0;
    for(i=0;i<MAXCOEFSMODEL;i++) {
      paramet.thet[i] = 0.0;
    }
  }

  // Used to compute the window caracteristics
  center_of_gravity(&pyr_support[0], label,
		    &ligravbase, &cogravbase, &paramet.fen, 0);

  double row, col;
  model.getParameters(paramet.thet);
  model.getOrigin(row, col);

  if ((row == -1.f) && (col == -1.f)) { // Origin  not specified.
    // Origin set to the center of gravity
    paramet.li_c = ligravbase;
    paramet.co_c = cogravbase;
    model.setOrigin(paramet.li_c, paramet.co_c);
  }
  else {
    paramet.li_c = row;
    paramet.co_c = col;
  }

  if (DEBUG_LEVEL2) printf("avant appel RMRmod() label: %d\n", label);

  // Estimation robuste
  state = RMRmod(label, &paramet.fen, pyr_level_max,
		 pyramid1.pyr_ima, pyramid2.pyr_ima,
		 pyramid1.pyr_gx, pyramid1.pyr_gy,
		 pyramid2.pyr_gx, pyramid2.pyr_gy,
		 max_it_irls, ty_pond, least_mean_square,
		 type_variance, max_it_stab,
		 &paramet, para_init, &variance, Mode_Niv_Init,
		 level_ct, level_lin, level_quad,
		 final_est_level, &pyr_weights[0], tx_pts_min,
		 &pyr_fl1[0], &pyr_fl2[0],
		 &pyr_fl3[0], &pyr_support[0], verbose,
		 &ct_level_intro, &lin_level_intro, &quad_level_intro);

  // Test si l'estimation s'est bien deroulee
  if (state == false) return false;

  // Warning:
  // pyr_weights[final_est_level].ad contains the weights

  //
  // Compute the support size: conform pixels / pixels zone de recouvrement.
  //
  if (compute_support_size == 1) {
    double tx=0.0, ty=0.0;
    double a=0.0, b=0.0, c=0.0, d=0.0;
    double q1=0.0, q2=0.0, q3=0.0, q4=0.0, q5=0.0, q6=0.0; // Modele de mvt.

    int xsize = pyr_weights[final_est_level].nbco;
    int ysize = pyr_weights[final_est_level].nbli;
    int x, y;			// Coordonnees d'un pixel.
    int px, py;			// Coordonnees du pixel (x,y) deplace.
    float xg, yg;			// Origine du modele de mvt.
    int inter_size = 0;		// Taille de la zone commune entre t et t+1.
    float nb_pt_inter= 0.0;	// Nbre de points utiles dans la zone commune.
    float *pond = pyr_weights[final_est_level].ad;
    short *support = pyr_support[final_est_level].ad;
    int  support_size = 0;

    // On ramene si necessaire les parametres du modele de mvt du niveau 0
    // correspondant a la resolution la plus fine, au niveau final de
    // l'estimation (correspondant a l'imagette a un niveau plus grossier de la
    // pyramide).
    Para tmp_modele;

    change_level(&paramet, &tmp_modele, final_est_level);

    // The parameters
    tx = tmp_modele.thet[0];
    ty = tmp_modele.thet[1];
    a = tmp_modele.thet[2];
    b = tmp_modele.thet[3];
    c = tmp_modele.thet[4];
    d = tmp_modele.thet[5];
    q1 = tmp_modele.thet[6];
    q2 = tmp_modele.thet[7];
    q3 = tmp_modele.thet[8];
    q4 = tmp_modele.thet[9];
    q5 = tmp_modele.thet[10];
    q6 = tmp_modele.thet[11];

    // Balayage de l'imagette au niveau final de l'estimation
    xg = tmp_modele.co_c;
    yg = tmp_modele.li_c;
    // Pour optimiser legerement cette partie, 3 cas sont envisages en fonction
    // du degre du modele

    if ( model_degree(model.getIdModel()) == 2 ) {
      // Cas des modeles de mouvement quadratique
      for (y = 0; y < ysize; y++) {
	float y_yg    = y - yg;
        float y_yg2   = y_yg * y_yg;
	float y_yg_b  = y_yg * b;
	float y_yg_d  = y_yg * d;
	float ty_y    = ty + y;
	int    y_xsize = y * xsize;

	for (x = 0; x < xsize; x++) {
	  if (support[x + y_xsize] == label) {
	    float x_xg      = x - xg;
	    float x_xg2     = x_xg * x_xg;
	    float x_xg_y_yg = x_xg * y_yg;

	    support_size ++;

	    // Calcul du pixel deplace. Le calcul a effectuer est le suivant:
	    // px = (int) (tx + (x-xg)*a + (y-yg)*b
	    //  + (x-xg)*(x-xg)*q1 + (x-xg)*(y-yg)*q2 + (y-yg)*(y-yg)*q3 + x);
	    // py = (int) (ty + (x-xg)*c + (y-yg)*d
	    //  + (x-xg)*(x-xg)*q4 + (x-xg)*(y-yg)*q5 + (y-yg)*(y-yg)*q6 + y);
	    px = (int) (tx   + (x_xg)*a + y_yg_b
			+ (x_xg2)*q1 + (x_xg_y_yg)*q2 + (y_yg2)*q3 + x);
	    py = (int) (ty_y + (x_xg)*c + y_yg_d
			+ (x_xg2)*q4 + (x_xg_y_yg)*q5 + (y_yg2)*q6);

	    // Test si pixel deplace de t a t+1 reste a l'interieur de l'image
	    if ( (px >= 0) && (px < xsize) && (py >= 0) && (py < ysize) ) {
	      // Le pixel deplace restant dans l'imagette, incrementation de
	      // la taille de la zone commune entre t et t+1.
	      inter_size ++;
	      // Test si le pixel (x,y) participe au mvt dominant. En fait,
	      // on teste si la ponderation appliquee au pixel (x, y)
	      // est superieure a un seuil.
	      if (pond[x + y_xsize] > seuil_poids) {
		nb_pt_inter += 1.0;
	      }
	    }
	  }
	}
      }
    }
    else if ( model_degree(model.getIdModel()) == 1){
      // Cas des modeles de mouvement affine
      for (y = 0; y < ysize; y++) {
	float y_yg    = y - yg;
	float y_yg_b  = y_yg * b;
	float y_yg_d  = y_yg * d;
	float ty_y    = ty + y;
	int    y_xsize = y * xsize;

	for (x = 0; x < xsize; x++) {
	  if (support[x + y_xsize] == label) {
	    float x_xg = x - xg;

	    support_size ++;

	    // Calcul du pixel deplace. Le calcul a effectuer est le suivant:
	    //	px = (int) (tx + (x-xg)*a + (y-yg)*b + x);
	    //	py = (int) (ty + (x-xg)*c + (y-yg)*d + y);
	    px = (int) (tx   + (x_xg)*a + y_yg_b + x);
	    py = (int) (ty_y + (x_xg)*c + y_yg_d);

	    // Test si pixel deplace de t a t+1 reste a l'interieur de l'image
	    if ( (px >= 0) && (px < xsize) && (py >= 0) && (py < ysize) ) {
	      // Le pixel deplace restant dans l'imagette, incrementation de
	      // la taille de la zone commune entre t et t+1.
	      inter_size ++;
	      // Test si le pixel (x,y) participe au mvt dominant. En fait,
	      // on teste si la ponderation appliquee au pixel (x, y)
	      // est superieure a un seuil.
	      if (pond[x + y_xsize] > seuil_poids) {
		nb_pt_inter += 1.0;
	      }
	    }
	  }
	}
      }
    }
    else if ( model_degree(model.getIdModel()) == 0) {
      // Cas des modeles de mouvement constant
      for (y = 0; y < ysize; y++) {
	float ty_y    = ty + y;
	int   y_xsize = y * xsize;

	for (x = 0; x < xsize; x++) {
	  if (support[x + y_xsize] == label) {
	    support_size ++;

	    // Calcul du pixel deplace. Le calcul a effectuer est le suivant:
	    //	px = (int) (tx + x);
	    //	py = (int) (ty + y);
	    px = (int) (tx + x);
	    py = (int) (ty_y );

	    // Test si pixel deplace de t a t+1 reste a l'interieur de l'image
	    if ( (px >= 0) && (px < xsize) && (py >= 0) && (py < ysize) ) {
	      // Le pixel deplace restant dans l'imagette, incrementation de
	      // la taille de la zone commune entre t et t+1.
	      inter_size ++;
	      // Test si le pixel (x,y) participe au mvt dominant. En fait,
	      // on teste si la ponderation appliquee au pixel (x, y)
	      // est superieure a un seuil.
	      if (pond[x + y_xsize] > seuil_poids) {
		nb_pt_inter += 1.0;
	      }
	    }
	  }
	}
      }
    } // fin else degre modele

    // Calcul du taux de points conformes dans la zone de recouvrement.
    // Si le support est inferieur a 1/8 de la taille de l'image, on met
    // le rapport a 0.
    if (inter_size < ( (int) (support_size) / 8) )
      rapport_poids = 0.0;
    else
      rapport_poids = nb_pt_inter / inter_size;
  } // Fin du calcul du taux de points dans la zone de recouvrement.

  // Mise a jour de la variance du residuel.
  sigma2res = paramet.sigma2res;

  if (compute_covariance) {
    // Updates the covariance matrix
    memcpy(covariance, paramet.covariance,
	   MAXCOEFSMODEL * MAXCOEFSMODEL * sizeof(double));
  }

  double coefs[MAXCOEFSMODEL-1];
  for (i=0; i < MAXCOEFSMODEL-1; i++)
    coefs[i] = paramet.thet[i];

  model.setOrigin(paramet.li_c, paramet.co_c);
  model.setParameters(coefs);
  model.setVarLight(model.getVarLight(), paramet.thet[12]);

  if (DEBUG_LEVEL1)
    cout << "Fin CMotion2DEstimator::estimate()" << endl;

  return true;
}

/*!

  Implements the estimation of a 2D parametric motion model.

  This method differs from estimate(const CMotion2DPyramid &, const
  CMotion2DPyramid &, const short *, short, CMotion2DModel &) only in the type
  of the estimation support \f$R_t\f$ which is here \e unsigned \e char.

  Return true if the estimation process succeeds, false otherwise.

  \sa estimate(const CMotion2DPyramid &, const CMotion2DPyramid &, const short *, short, CMotion2DModel &)

*/
bool CMotion2DEstimator::estimate(const CMotion2DPyramid & pyramid1,
				  const CMotion2DPyramid & pyramid2,
				  const unsigned char *support,
				  unsigned char label,
				  CMotion2DModel & model,
				  bool useModelAsInitialization)
{
  int nrows_pyr1, ncols_pyr1;
  int nrows_pyr2, ncols_pyr2;
  if (pyramid1.getNumberOfRows(nrows_pyr1) != PYR_NO_ERROR)
    return false;
  if (pyramid1.getNumberOfCols(ncols_pyr1) != PYR_NO_ERROR)
    return false;
  if (pyramid2.getNumberOfRows(nrows_pyr2) != PYR_NO_ERROR)
    return false;
  if (pyramid2.getNumberOfCols(ncols_pyr2) != PYR_NO_ERROR)
    return false;
  if ((nrows_pyr1 != nrows_pyr2) || (ncols_pyr1 != ncols_pyr2)) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DEstimator::estimate() \n\tError: Pyramid size differ");
    __panic(message);
    return false;
  }

  int size = nrows_pyr1 * ncols_pyr1;
  short *s_support = new short [ size ];
  bool state;

  // Initialize the estimation support
  cast_uchar_short(support, s_support, size);

  state = estimate(pyramid1, pyramid2, s_support, (short) label,
		   model, useModelAsInitialization);

  delete [] s_support;

  return (state);
}

/*!

  Returns a pointer to the map containing the weights \f$w_i\f$ computed within
  the robust multiresolution estimation. The dimension of this map depends on
  the final estimation level considered in the pyramids. The number of rows and
  columns of the weight map is given respectively by \e n_rows and \e n_cols.

  The weight values are in [0; 1]. A value close to 1 indicates that the
  corresponding pixel is conforming to the dominant motion. A value close to 0
  indicates that the pixel is considered as an outlier.

  This method is to use after a call to estimate().

  \sa estimate()

*/
float *CMotion2DEstimator::getWeights(int &n_rows, int &n_cols)
{

  n_rows = pyr_weights[final_est_level].nbli;
  n_cols = pyr_weights[final_est_level].nbco;

  return pyr_weights[final_est_level].ad;
}

/*!

  Print an error message.

*/
void CMotion2DEstimator::__panic(char *message)
{
  cerr << message << endl;
}

  float *CMotion2DEstimator::getDisplacedRowsSpatialGradientDataAddress(int level, int &n_rows, int &n_cols)
{
   if (init_mem_estIsDone == false)
     {
       //       error = PYR_NOT_ALLOCATED;
       cout << "Error level "<<endl;
       exit(0);
       return 0;
     }

    if (level < pyr_level_max && level>=0)
    {
      n_rows = pyr_fl2[level].nbli;
      n_cols = pyr_fl2[level].nbco;

      return pyr_fl2[level].ad;
    }

    //    error = PYR_NO_ERROR;
    cout << "Error level"<<endl;
    exit(0);

    return 0;

}
  float *CMotion2DEstimator::getDisplacedColsSpatialGradientDataAddress(int level, int &n_rows, int &n_cols)
{
   if (init_mem_estIsDone == false)
     {
       //       error = PYR_NOT_ALLOCATED;
       cout << "Error level "<<endl;
       exit(0);
       return 0;
     }

    if (level < pyr_level_max && level>=0)
    {

      n_rows = pyr_fl1[level].nbli;
      n_cols = pyr_fl1[level].nbco;

      return pyr_fl1[level].ad;
    }

    //    error = PYR_NO_ERROR;
    cout << "Error level"<<endl;
    exit(0);

    return 0;
}

/*!

   Get the address of the DFD map for the level given as argument.
   Updates the dimension of the map.

*/
float *CMotion2DEstimator::getDFDDataAddress(int level, int &n_rows, int &n_cols)
{
   if (init_mem_estIsDone == false)
     {
       //       error = PYR_NOT_ALLOCATED;
       cout << "Error level "<<endl;
       exit(0);
       return 0;
     }
    if (level < pyr_level_max && level>=0)
    {
      n_rows = pyr_fl3[level].nbli;
      n_cols = pyr_fl3[level].nbco;

      return pyr_fl3[level].ad;
   }

    //    error = PYR_NO_ERROR;
    cout << "Error level"<<endl;
    exit(0);

    return 0;

}
