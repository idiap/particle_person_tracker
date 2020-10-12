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
  \file CMotion2D.h
  \brief File to include to use CMotion2D.
*/

#ifndef CMotion2D_h
#define CMotion2D_h

// Include for Motion2D
#include "CMotion2DImage.h"
#include "CMotion2DEstimator.h"
#include "CMotion2DPyramid.h"
#include "CMotion2DModel.h"
#include "CMotion2DWarping.h"
#include "CReader.h"
#include "CWriter.h"


#define GETOPTARGS "a:b:c:d:e:f:ghi:j:k:l:m:n:o:p:q:r:s:t:u:vw:x:y:z?C:F:IR:"
#define max(a,b) (a>b?a:b)

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



class MOTION2D_API CMotion2D {

 public:
  CMotion2D();
  ~CMotion2D();
  void printHeaderResults(FILE *output, CMotion2DModel model,
			  double multfactor);
  void printResults(FILE *output, unsigned long i1, unsigned long i2,
		    CMotion2DModel model, double support_size,
		    unsigned nbsubsample, double multfactor);
  void printVresResults(FILE * output, CMotion2DImage<float> &Vres,
			CMotion2DImage<unsigned char> &Map, string dpath,
			int frame);
  int  getoption (int argc, char** argv, char* pszValidOpts, char** ppszParam);
  void getoptions(int argc, char **argv);
  int  main(int argc, char **argv);
  int  testReader(int argc, char **argv);
  void usage(char *name, char *badparam);
  void setReader(CReader *);
  void getVres(CMotion2DImage<float>&,CMotion2DImage<unsigned char>&, int);
  void free();
  unsigned getNbSubsample();
  CMotion2DModel &getMotion2DModel();

 private:
  CMotion2DImage<short> I;		// Image
  CMotion2DImage<unsigned char> S;	// Motion estimator support
  CMotion2DImage<short> B;		// Backwarped image
  CMotion2DImage<unsigned char> W;	// M-estimator weights image
  CMotion2DEstimator	estimator;	// Motion estimator
  CMotion2DModel	model;		// Parametric motion model
  CMotion2DPyramid	pyramid1;	// Pyramid on image 1
  CMotion2DPyramid	pyramid2;	// Pyramid on image 2
  CMotion2DWarping	warping;	// Warping

  string ipath ;		// Image path
  string filename;		// Complete filename for an image of the video
  string rpath;			// Result filename to store the model
  string opath;			// Result filename for residual motion
  string bpath;			// Back-warped image path
  string wpath;			// M-estimator weights image path
  string dpath;			// Residual motion image path
  string spath;			// Estimation support path
  string Fpath;			// Optic flow path

  long unsigned niter ;		// Number of images to process
  int  step;			// Step between 2 images
  unsigned char label;		// Value of the motion estimator support
  int slabel;			// Value of the motion estimator support
  string model_id;		// Parametric motion model ID to estimate
  bool var_light;		// Lighting variation parameter estimation
  bool model_orig_fixed;	// Indicates if an origin is fixed
  double model_row_orig;	// Motion model origin (line coordinate)
  double model_col_orig ;	// Motion model origin (column coordinate)
  unsigned pyr_nlevels;		// Number of levels in a multi-resolution pyr
  unsigned pyr_stop_level;	// Pyramid level where the estimator stops
  bool verbose;			// Verbose mode
  bool support_empty;		// Indicates if the support is empty.

  int b_ncols, b_nrows;		// Back-warped image size
  int b_col_orig, b_row_orig;	// Back-warped origin in image frame
  bool compute_covariance;	// Flag to compute the covariance matrix
  unsigned nbsubsample;		// Images spatial resolution: 0 full, 1 mid...

  unsigned long frame;		// Current frame number processed
  double multfactor;		// Multiplier factor for motion parameters
  unsigned raw_nrows;		// Number of rows for RAW8 or RAW16 images
  unsigned raw_ncols;		// Number of cols for RAW8 or RAW16 images
  bool useModelAsInitialization; // How is the estimated model init

  CReader *Ireader;		// Image reader
  CReader *Sreader;		// Support reader
  CWriter *writer;		// Image writer
};

#endif
