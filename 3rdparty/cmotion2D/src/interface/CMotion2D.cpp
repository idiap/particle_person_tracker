/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <cmotion2d/CMotion2D.h>

#include <cmotion2d/CImageReader.h>
#ifndef __NO_IMAGEIO_MPEG_
#  include <cmotion2d/CMpeg2Reader.h>
#endif
#include <cmotion2d/CImageWriter.h>
#include <cmotion2d/FieldVector.h>
#include <cstring>

/*!
  \file CMotion2D.cpp
  \brief Definition of the CMotion2D class.
*/

/*!

  \class CMotion2D

  \brief The CMotion2D class implements the Motion2D software.

*/

/*!

  2D parametric motion estimator constructor.

*/
CMotion2D::CMotion2D()
{
  frame = 1;			// Frame to process
  step  = 1;			// Step between 2 images
  model_id.assign("AC");	// Parametric motion model ID to estimate
  var_light = false;		// Lighting variation parameter estimation
  model_orig_fixed = false;	// Indicates if an origin is fixed
  model_row_orig = -1.;		// Motion model origin (line coordinate)
  model_col_orig = -1.;		// Motion model origin (column coordinate)
  pyr_nlevels = 4;		// Number of levels in a multi-resolution pyr
  pyr_stop_level = 0;		// Pyramid level where the estimator stops
  verbose = false;		// Verbose mode
  support_empty = false;	// Indicates if the support is empty.
  b_ncols=-1; b_nrows=-1;	// Back-warped image size
  b_col_orig=0; b_row_orig=0;	// Back-warped origin in image frame
  compute_covariance = false;	// Flag to compute the covariance matrix
  label = 234;			// Value of the motion estimator support
  slabel = -1;			// Value of the motion estimator support
  nbsubsample = 0;		// Number of subsampling to apply
  multfactor = 1.0;		// Multiplier factor for motion parameters

  raw_nrows = 0;		// Number of rows for RAW8 or RAW16 images
  raw_ncols = 0;		// Number of cols for RAW8 or RAW16 images
  useModelAsInitialization = false; // How is the estimated model init

  Ireader = NULL;		// Image reader
  Sreader = NULL;		// Support reader
  writer  = NULL;		// Image writer
}


/*!

  Find a char in a string.

  \return true if the char was find, false otherwise.

*/
bool findChar(string ch, char *f)
{
  int n = ch.find(f);
  int size = ch.size();
  return (n>0 && n<size);
}

/*!

  Process the motion estimation.

*/
int CMotion2D::main(int argc, char **argv)
{
  bool ierr= true;
  FILE * rfile=0;
  FILE * ofile=0;
  getoptions(argc, argv);  // Initialize the parameters


  /**************************************************************************/
  //Modif Francois Coldefy
  /*************************************************************************/

  // initialize the reader depending on the file type: mpeg or image sequence
#ifndef __NO_IMAGEIO_MPEG_
  CMpeg2Reader mpegr;
#endif
  CImageReader imgr;
#ifndef __NO_IMAGEIO_MPEG_
  if (findChar(ipath, _mpg) || findChar( ipath, _mpeg))
    Ireader = &mpegr;
  else
#endif
    Ireader = &imgr;

  /**************************************************************************/

  Ireader->setFileName(ipath);
  Ireader->setFrameNumber(frame);
  filename = Ireader->getFileName();
  CReader::EReaderFormat format = Ireader->getFormat();
  // Test RAW format specific options
  if (format == CReader::FORMAT_RAW8 || format == CReader::FORMAT_RAW16) {
    if (raw_nrows == 0 || raw_ncols == 0) {
      cout << endl
	   << "Error: When using option -p to specify RAW images path, "
	   << endl
	   << "you have to specify the images size with option -R and -C."
	   << endl;
      exit(0);
    }
  }

  if (!Ireader->openStream()) {
    cout <<"Can't open stream "<< ipath <<endl;
    exit(-1);
  }

  cout << "Motion estimation in process....." << endl;

  ierr = Ireader->getFrame(I, nbsubsample);
  if (ierr == false) {
    cout <<" Error: Can't access to the frame..." << endl;
    exit(-1);
  }

  // Set the support format by regarding the image extension
  if (!spath.empty()) { // Otherwise the support consist of the whole image
#ifndef __NO_IMAGEIO_MPEG_
    if (findChar(spath, _mpg) || findChar(spath, _mpeg))
      Sreader = &mpegr;
    else
#endif
      Sreader = &imgr;
    Sreader->setFileName(spath);
    Sreader->setFrameNumber(frame);
    if (!Sreader->openStream()) {
      cout <<"Can't open stream "<< spath <<endl;
      exit(-1);
    }
  }

  // Set the image writer
  CImageWriter imgw;
  writer = &imgw;

  // Initialisation of the 2D parametric motion model to estimate
  model.reset();	// Set all the parameters to zero
  model.setIdModel(model_id); // Set the model id
  model.setVarLight(var_light); // Set the illumination variation estimation

  // Initialize the model origin
  if ( ! model_orig_fixed) {
    // Origin fixed at the middle of the image
    model_row_orig = I.GetRows() / 2.0;
    model_col_orig = I.GetCols() / 2.0;
  }

  model.setOrigin(model_row_orig, model_col_orig); // Set the origin

  // Pyramids allocation for two successive images
  pyramid1.allocate(I.GetRows(), I.GetCols(), pyr_nlevels);
  pyramid2.allocate(I.GetRows(), I.GetCols(), pyr_nlevels);

  // Fill the pyramid 1 whith image 1
  pyramid1.build(I.bitmap);

  // Initialize the motion estimator
  // Set the pyramid start estimation level
  ierr = estimator.setFirstEstimationLevel(pyr_nlevels - 1);
  if (ierr == false) {
    exit(-1);
  }
  // Set the pyramid stop estimation level
  ierr = estimator.setLastEstimationLevel(pyr_stop_level);
  if (ierr == false) {
    exit(-1);
  }
  estimator.setRobustEstimator(true);

  if (compute_covariance)
    estimator.computeCovarianceMatrix(true);

  // Initialize the support
  if (spath.empty()) {
    // Set the estimator support to the whole image
    S.Init(I.GetRows(), I.GetCols(), label);
  }
  else {
    // Set the support label value
    label = (unsigned char) slabel;
  }


  if ( !bpath.empty() )
  {
    // Set the backwarped image size
    //B.Init(I.GetRows() +40, I.GetCols() +200 );
    // Init warping by positionning the reference image in the backwarped img.
    //warping.initBackWarp(B.GetRows(), B.GetCols(), -20, -20);

    // Set the backwarped image size
    if (b_nrows <= 0) b_nrows = I.GetRows();
    if (b_ncols <= 0) b_ncols = I.GetCols();
    B.Init(b_nrows, b_ncols);
    warping.initBackWarp(B.GetRows(), B.GetCols(), b_row_orig, b_col_orig);

    // Backwarp the first image, taken as image reference
    warping.backWarp(I.bitmap, I.GetRows(), I.GetCols(), B.bitmap, model);
    // Construct the backwarped image filename
    writer->setFileName(bpath);
    writer->setFrameNumber(frame);
    filename = writer->getFileName();
    if (verbose)
      cout << "\tWrite backwarped image: " << filename << endl ;
    ierr = writer->writeFrame(B);      // Write the backwarped image
    if (ierr == false) {
      cout << "Can not write the image: " << filename << endl;
      exit(-1);
    }
  }

  // Open the result file
  if ( !rpath.empty() ) {
    rfile = fopen(rpath.c_str(), "wb");

    printHeaderResults(rfile, model, multfactor);
  }

  if ( !opath.empty() ) {
    ofile = fopen(opath.c_str(), "wb");
  }


  // Robust estimation loop
  for (long unsigned n=1; n <= niter; n++) {

    // Initialize the support
    if (!spath.empty()) {
      // Otherwise the support consist of the whole image
      Sreader->setFileName(spath);
      Sreader->setFrameNumber(frame);
      filename = Sreader->getFileName();
      if (verbose)
	cout << "Load support: " << filename << endl;
      ierr = Sreader->getFrame(S, nbsubsample);
      if (ierr == false) {
	cout << "Can not read the support image: " << filename << endl;
	exit(-1);
      }

      // Test if the support is empty
      support_empty = true;
      for (unsigned i=0; i < S.GetRows() * S.GetCols(); i++) {
	if (S.bitmap[i] == label) {
	  support_empty = false;
	  break;
	}
      }

      if (support_empty) {
	if (verbose)
	  cout << "\tWarning: the support is empty, motion estimation not done"
	       << endl;
      }
    }

    // Update the image frame number
    frame += step;

    // Load the next image (image 2)
    Ireader->setFileName(ipath);
    Ireader->setFrameNumber(frame);
    filename = Ireader->getFileName();
    if (verbose)
      cout << "Load image: " << filename << endl ;
    ierr = Ireader->getFrame(I, nbsubsample);
    if (ierr == false) {
      cout << "Can not read the image: " << filename << endl;
      exit(-1);
    }

    // Fill the pyramid 2 with image 2
    pyramid2.build(I.bitmap);
    // 2D Parametric motion estimation
    if ( ! support_empty) {
      bool state;
      state = estimator.estimate(pyramid1, pyramid2, S.bitmap, label,
				 model, useModelAsInitialization);

      if (state == false){
	//pyramid1.destroy();
	//pyramid2.destroy();
	//exit(0);
	cout << "\nError during motion estimation. " << endl
	     << "Possible reason: not enought gradient." << endl
	     << "The model is set to zero..." << endl << endl;
      }
    }

    // Exchange pyramid 1 and 2
    pyramid1.exchange(pyramid2);

    if (support_empty) {
      continue;
    }

    // Print results
    double support_size;
    if (! estimator.getSupportSize(support_size)) support_size = -1.;
    if (verbose) {
      printResults(stdout, frame - step, frame, model,
		   support_size, nbsubsample, multfactor);
      if ( estimator.isResidualVarianceComputed() )
	cout << "\tResidual variance: "
	     << estimator.getResidualVariance() << endl;

      if ( estimator.isCovarianceMatrixComputed() ) {
	int cov_nrows, cov_ncols;
	double *cov = estimator.getCovarianceMatrix(cov_nrows, cov_ncols);
	cout << "\tCovariance matrix: " <<  endl;
	for (int i=0; i < cov_nrows; i ++) {
	  cout << "\t";
	  for (int j=0; j < cov_ncols; j ++) {
	    fprintf(stdout, "%9.3g ", cov[i*cov_ncols + j]);
	    //cout << cov[i*cov_ncols + j] << " ";
	  }
	  cout << endl;
	}
      }
    }

    if ( !rpath.empty() ) {
      if (verbose)
	cout << "\tUpdate the result file: " << rpath
	     << " frames " << frame-step << ", "<<frame<< endl;
      printResults(rfile, frame - step, frame, model,
		   support_size, nbsubsample, multfactor);
    }


    if ( !bpath.empty() ) {
      // Backwarp the current image in the first image reference frame
      warping.backWarp(I.bitmap, I.GetRows(), I.GetCols(), B.bitmap, model);
      // Construct the backwarped image filename
      writer->setFileName(bpath);
      writer->setFrameNumber(frame);
      filename = writer->getFileName();
      if (verbose)
	cout << "\tWrite backwarped image: " << filename << endl ;
      ierr = writer->writeFrame(B);    // Write the backwarped image
      if (ierr == false) {
	cout << "Can not write the backwarped image: " << filename << endl;
	exit(-1);
      }
    }

    if ( !wpath.empty() ) {
      float *weights = NULL;
      int n_rows = 0, n_cols = 0;
      weights = estimator.getWeights(n_rows, n_cols); // weights are in [0,1]

      W.Init(n_rows, n_cols);
      for (int i=0; i < n_rows; i++)
	for (int j=0; j < n_cols; j++)
	  W[i][j]= (unsigned char) (weights[i*n_cols + j] * 255);

      // Construct the M-estimator weights image filename
      writer->setFileName(wpath);
      writer->setFrameNumber(frame - step);
      filename = writer->getFileName();
      if (verbose)
	cout << "\tWrite M-estimator weights image: " << filename << endl;
      ierr = writer->writeFrame(W);
      if (ierr == false) {
	cout << "Can not write the weights image: " << filename << endl;
	exit(-1);
      }
    }

    if ( !Fpath.empty() ) {
      // Construct the field vector image filename
      writer->setFileName(Fpath);
      writer->setFrameNumber(frame - step);
      filename = writer->getFileName();
      if (verbose)
      	cout << "\tWrite field vector image: " << filename << endl ;

      // Compute and save the displacement vector in all pixels
      ierr = WriteFieldVector(model, S, label, filename.c_str());
      if (ierr == false) {
	cout << "Can not write the field vector image: " << filename << endl;
	exit(-1);
      }
    }

    /***********************************************************************/
    // Modif Francois Coldefy
    /***********************************************************************/
    if ( !dpath.empty() || !opath.empty()) {
      {
	CMotion2DImage<unsigned char>  Map;
	CMotion2DImage<float>  Vres;// M-estimator residualMotion
	getVres(Vres, Map, 0);
	printVresResults(ofile,Vres,Map,dpath,frame-step);
      }
      if ((frame/100)*100 == frame)
	cout <<" current image "<<frame<<endl;
    }
  }

  if ( !rpath.empty() )
    fclose(rfile);
  if ( !opath.empty() )
    fclose(ofile);

  Ireader->closeStream();
  if (!spath.empty()) {
    Sreader->closeStream();
  }
  return 0;
}


/*!

  Free all the memory allocated for the image pyramids.

*/
void CMotion2D::free()
{
  pyramid1.destroy();
  pyramid2.destroy();
}


/*!

  Computes the residual.

*/
void CMotion2D::getVres(CMotion2DImage<float> &Vres,
			CMotion2DImage<unsigned char> &Map, int level)
{

  int ysize ;
  int xsize ;
  float *dfd,*gx,*gy,*gn;
  TPyramidError error;
  dfd = estimator.getDFDDataAddress(level,ysize,xsize);
  gx =  pyramid1.getColsSpatialGradientDataAddress(error, level);
  gy =  pyramid1.getRowsSpatialGradientDataAddress(error, level);

  int n= ysize*xsize;
  gn = new float[n];
  for (int k=0;k<n;k++)
    gn[k]=gx[k]*gx[k]+gy[k]*gy[k];

  CMotion2DImage<float > G,DFD,N;
  Vres.Init(ysize, xsize);
  Map.Init(ysize, xsize);
  G.Init(ysize, xsize);
  N.Init(ysize, xsize);
  DFD.Init(ysize, xsize);


  for (int y=0; y < ysize; y++)
  {
    int yy = y*xsize;
    for (int  x=0; x < xsize; x++)
    {
      Vres[y][x]=0;
      Map[y][x]=0;
      DFD[y][x]=fabs((double)dfd[yy + x]);
      G[y][x]=gn[yy + x];
      N[y][x]=sqrt((double)gn[yy + x]);
    }
  }

  delete []gn;

  CMotion2DModel tmpModel;
  tmpModel =model.getModelLevel(level);
  double row, col;
  double parameters[CMotion2DModel::MDL_NMAX_COEF];
  tmpModel.getOrigin(row, col);
  tmpModel.getParameters(parameters);
  double tx=0.0, ty=0.0;
  double a=0.0, b=0.0, c=0.0, d=0.0;
  double q1=0.0, q2=0.0, q3=0.0, q4=0.0, q5=0.0, q6=0.0; // Modele de mvt.


  tx = parameters[0];
  ty = parameters[1];
  a = parameters[2];
  b = parameters[3];
  c = parameters[4];
  d = parameters[5];
  q1 = parameters[6];
  q2 = parameters[7];
  q3 = parameters[8];
  q4 = parameters[9];
  q5 = parameters[10];
  q6 = parameters[11];


  // Balayage de l'imagette au niveau final de l'estimation
  double xg = col;
  double yg = row;
  // Pour optimiser legerement cette partie, 3 cas sont envisages en fonction
  // du degre du modele
  double px, py;

  int fen=1;
  float epsilon=10;
  bool t;
  int r=0;
  for (int y = 0; y < ysize; y++) {
    for (int x = 0; x < xsize; x++)
    {
      r++;
      float u=0,v=0;
      int count=0;
      t = false;
      for (int k= - fen;k<=fen;k++) {
	for (int l= -fen ; l<=fen; l++)
	{
	  int yk=y+k;
	  int xl=x+l;
	  if (yk>=0 && yk<ysize && xl<xsize && xl>=0)
	  {

	    float y_yg    = yk - yg;
	    float y_yg2   = y_yg * y_yg;

	    float x_xg      = xl - xg;
	    float x_xg2     = x_xg * x_xg;
	    float x_xg_y_yg = x_xg * y_yg;



	    // Calcul du pixel deplace. Le calcul a effectuer est le suivant:
	    // px = (int) (tx + (x-xg)*a + (y-yg)*b
	    //  + (x-xg)*(x-xg)*q1 + (x-xg)*(y-yg)*q2 + (y-yg)*(y-yg)*q3 + x);
	    // py = (int) (ty + (x-xg)*c + (y-yg)*d
	    //  + (x-xg)*(x-xg)*q4 + (x-xg)*(y-yg)*q5 + (y-yg)*(y-yg)*q6 + y);
	    px =  (tx   + (x_xg)*a + y_yg*b
		   + (x_xg2)*q1 + (x_xg_y_yg)*q2 + (y_yg2)*q3 + xl);
	    py =  (ty + (x_xg)*c + y_yg*d
		   + (x_xg2)*q4 + (x_xg_y_yg)*q5 + (y_yg2)*q6 + yk);
	    if (px>=0. && px<xsize && py>=0.0 && py<ysize)
	      // Test si pixel deplace de t a t+1 reste a l'interieur de l'image
	      if ( (px >= 0) && (px < xsize) && (py >= 0) && (py < ysize) ) {

		{
		  u+=DFD[yk][xl]*N[yk][xl];
		  v+=G[yk][xl];
		  count++;
		  if (k ==0 && l ==0)
		    t = true;

		}
	      }
	  }
	}
      }
      if ( t)
      {
	if (v<epsilon*epsilon*count)
	  v=epsilon*epsilon*count;

	Vres[y][x] =  u/v;

	Map[y][x] = 1;
      }
    }
  }
}

/*!

  Save the residual map.

*/
void CMotion2D::printVresResults(FILE * output, CMotion2DImage<float> &Vres,
				 CMotion2DImage<unsigned char> &Map,
				 string dpath, int frame)
{

  int n_rows  = Vres.GetRows();
  int n_cols  = Vres.GetCols();
  int surface=0;
  int n0=0;
  int n1=0;
  double threshold0 =0.1;
  double threshold = 4;

  for (int i=0; i < n_rows; i++) {
    for (int j=0; j < n_cols; j++)
    {
      if (Map[i][j]==1)
      {
	surface++;
	if (Vres[i][j]<threshold0)
	  n0++;
	else
	  n1++;
      }
      double u = Vres[i][j]*255/threshold;
      if (u>255)
	u=255;
      Vres[i][j]=(unsigned char) u;
    }
  }
  //  cout <<" d0 "<< ((double) n0)/((double)(n0+n1)) << " s" <<((double) surface)/((double)(n_rows*n_cols))<<endl;



  if (!dpath.empty())
  {
    CMotion2DImage<unsigned char> V;
    V.Init(n_rows, n_cols);
    for (int i=0; i < n_rows; i++)
      for (int j=0; j < n_cols; j++)
	V[i][j]=(unsigned char) Vres[i][j];
    writer->setFileName(dpath);
    writer->setFrameNumber(frame - step);
    filename = writer->getFileName();
    if (verbose)
      cout << "\tWrite residual image: " << filename << endl ;
    bool ierr = writer->writeFrame(V);
    if (ierr == false) {
      cout << "Can not write the residual image: " << filename << endl;
      exit(-1);
    }
  }
  if (output!=0)
    fprintf(output,"%d %d %d %d %d %d %d\n", frame, n0, n1, n1, n0, n1, n1);
}

/*!

  Motion estimator destructor.

*/
CMotion2D::~CMotion2D()
{
  free();
}

/*!

  Set the program options.

*/
void CMotion2D::getoptions(int argc, char **argv)
{
  char *optarg;
  int	c;
  if (argc <3)
    usage(argv[0], NULL);
  while ((c = getoption(argc, argv, (char*)GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'a': spath = optarg; break;
    case 'b': bpath = optarg; break;
    case 'c': slabel = atoi(optarg); break;
    case 'd': dpath = optarg; break;
    case 'e': multfactor = (double) atof(optarg); break;
    case 'f': frame = (unsigned long) atoi(optarg); break;
    case 'g': var_light = true; break;
    case 'h': usage(argv[0], NULL); break;
    case 'i': niter = atoi(optarg); break;
    case 'j': b_nrows = atoi(optarg); break;
    case 'k': b_ncols = atoi(optarg); break;
    case 'l': pyr_stop_level = atoi(optarg); break;
    case 'm': model_id = optarg; break;
    case 'n': pyr_nlevels = atoi(optarg); break;
    case 'o': opath = optarg; break;
    case 'p': ipath = optarg; break;
    case 'q': nbsubsample = (unsigned) atoi( optarg); break;
    case 'r': rpath = optarg; break;
    case 's': step = atoi(optarg); break;
    case 't': b_row_orig = atoi(optarg); break;
    case 'u': b_col_orig = atoi(optarg); break;
    case 'v': verbose = true; break;
    case 'x': model_orig_fixed = true; model_col_orig = atoi(optarg); break;
    case 'y': model_orig_fixed = true; model_row_orig = atoi(optarg); break;
    case 'w': wpath = optarg; break;
    case 'z': compute_covariance = true; break;
    case '?': usage(argv[0], NULL); break;
    case 'C': raw_ncols = (unsigned)atoi(optarg); break;
    case 'F': Fpath = optarg; break;
    case 'I': useModelAsInitialization = true; break;
    case 'R': raw_nrows = (unsigned)atoi(optarg); break;

    default:  usage(argv[0], NULL); break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    fprintf(stderr, "Bad argument %s\n", optarg);
    exit(0);
  }

  // Some option tests
  if ( !rpath.empty() ) {
    FILE *rfile = fopen(rpath.c_str(), "wb");
    if (rfile == NULL) {
      cout << endl
	   << "Bad argument -r: "
	   << "Cannot create the result filename specified "
	   << endl
	   << "with this option."
	   << endl;
      exit(0);
    }
    fclose(rfile);
  }

  if ( !spath.empty() ) {
    // Test if the support label is set
    if (slabel == -1) {
      cout << endl
	   << "Error: When using option -a to specify a support image path, "
	   << endl
	   << "you have to specify the support label too with option -c"
	   << endl;
      exit(0);
    }
  }

  if (slabel != -1) {
    // Test if the support image path is set
    if ( spath.empty() ) {
      cout << endl
	   << "Error: When using option -c to specify a support label, "
	   << endl
	   << "you have to specify the support image path too with option -a"
	   << endl;
      exit(0);
    }
  }

  if (pyr_nlevels < 1) {
    // The pyramids can not be build
    cout << endl
	 << "Error: When using option -n to specify the number of levels"
	 << endl
	 << "in the pyramids. This number must be greater than zero. "
	 << endl;
    exit(0);
  }

  if ( bpath.empty() ) {
    bool ommit = false;
    if (b_nrows != -1) {
      cout << "Warning: option -j ignored. " << endl;
      ommit = true;
    }
    if (b_ncols != -1) {
      cout << "Warning: option -k ignored. " << endl;
      ommit = true;
    }
    if (b_row_orig != 0) {
      cout << "Warning: option -t ignored. " << endl;
      ommit = true;
    }
    if (b_col_orig != 0) {
      cout << "Warning: option -u ignored. " << endl;
      ommit = true;
    }
    if (ommit == true)
      cout << "To take it into account don't ommit option -b." << endl << endl;
  }
}

/*!

Get next command line option and parameter

\param argc Count of command line arguments
\param argv Array of command line argument strings
\param pszValidOpts String of valid, case-sensitive option characters,
a colon ':' following a given character means that
option can take a parameter
\param ppszParam Pointer to a pointer to a string for output

\return
- If valid option is found, the character value of that option
  is returned, and *ppszParam points to the parameter if given,
  or is NULL if no param.
- If standalone parameter (with no option) is found, 1 is returned,
  and *ppszParam points to the standalone parameter.
- If option is found, but it is not in the list of valid options, -1 is
  returned, and *ppszParam points to the invalid argument When end of argument
  list is reached, 0 is returned, and *ppszParam is NULL.
*/
int CMotion2D::getoption(int argc,
			 char** argv,
			 char* pszValidOpts,
			 char** ppszParam)
{
  static int iArg = 1;
  int chOpt;
  char* psz = NULL;
  char* pszParam = NULL;

  if (iArg < argc) {
    psz = &(argv[iArg][0]);
    if (*psz == '-') { // || *psz == '/')  {
      // we have an option specifier
      chOpt = argv[iArg][1];
      if (isalnum(chOpt) || ispunct(chOpt)) {
	// we have an option character
	psz = strchr(pszValidOpts, chOpt);
	if (psz != NULL) {
	  // option is valid, we want to return chOpt
	  if (psz[1] == ':') {
	    // option can have a parameter
	    psz = &(argv[iArg][2]);
	    if (*psz == '\0') {
	      // must look at next argv for param
	      if (iArg+1 < argc) {
		psz = &(argv[iArg+1][0]);
		// next argv is the param
		iArg++;
		pszParam = psz;
	      }
	      else {
		// reached end of args looking for param
	      }

	    }
	    else {
	      // param is attached to option
	      pszParam = psz;
	    }
	  }
	  else {
	    // option is alone, has no parameter
	  }
	}
	else {
	  // option specified is not in list of valid options
	  chOpt = -1;
	  pszParam = &(argv[iArg][0]);
	}
      }
      else {
	// though option specifier was given, option character
	// is not alpha or was was not specified
	chOpt = -1;
	pszParam = &(argv[iArg][0]);
      }
    }
    else {
      // standalone arg given with no option specifier
      chOpt = 1;
      pszParam = &(argv[iArg][0]);
    }
  }
  else {
    // end of argument list
    chOpt = 0;
  }

  iArg++;
  *ppszParam = pszParam;
  return (chOpt);
}


/*!

Print the program options.

*/
void CMotion2D::usage(char *name, char *badparam)
{
  if (badparam)
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);

  fprintf(stdout, "\n\
  Copyright (c) 1995-2005 by IRISA/INRIA Rennes.\n\
  All Rights Reserved.\n\
\n\
  This software was developed at:\n\
  IRISA/INRIA Rennes\n\
  Campus Universitaire de Beaulieu \n\
  35042 Rennes Cedex\n\
\n\
  http://www.irisa.fr\n\
\n\
SYNOPSIS\n\
  %s [-p image or video path] [-R rows number for RAW images]\n\
  [-C columns number for RAW images] [-f first frame]\n\
  [-s step] [-i iterations] [-m model id] [-g] [-x model col orig]\n\
  [-y model row orig] [-a support image or video path] [-z] \n\
  [-c support label] [-q nbsubsample] [-n number of pyramid levels] \n\
  [-l pyramid stop level] [-r dominant motion filename] \n\
  [-r multiplier factor] [-b back warped image path] \n\
  [-j back warped image nrows] [-k back warped image ncols] \n\
  [-t back warped image row origin] [-u back warped image col origin] \n\
  [-w weights image path] [-F optic flow field]  \n\
  [-o residual motion filename] [-d residual motion image path] \n\
  [-I] [-v] [-h] [-?]\n\
\n\
DESCRIPTION\n\
  The Motion2D software provides a method to estimate 2D parametric\n\
  motion models between two successive images. It can handle several\n\
  types of motion models, respectively, constant model (translation),\n\
  affine, and quadratic models. Moreover, it integrates the possibi-\n\
  lity of taking into account the global variation of illumination.\n\
  Motivations for the use of such models are, on one hand, their\n\
  efficiency, which has been demonstrated in numerous contexts such\n\
  as estimation, segmentation, tracking, and interpretation of motion, \n\
  and on the other hand, their low conputational cost compared to \n\
  optical flow estimation. Moreover to have the best accuracy for the\n\
  estimated parameters, and to take into account the problem of mul-\n\
  tiple motion, Motion2D exploit a robust, multiresolution and incre-\n\
  mental estimation method exploiting only the spatio-temporal deri-\n\
  vatives of the intensity function.\n", name);

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
  Motion2D can handle Mpeg2, PNG, PNM and RAW image file formats. \n\
  The different PNM formats are PGM (P5) and PPM (P6). A RAW image \n\
  file contains simply the list of pixel values with no header. \n\
  Two sets of RAW format are supported, one for the traditional \n\
  8-bits images nammed RAW8, the other for 9 to 16 bits images \n\
  named RAW16.\n");
#else
  fprintf(stdout, "\n\
  Motion2D can handle Mpeg2, PNM and RAW image file formats. \n\
  The different PNM formats are PGM (P5) and PPM (P6). A RAW image \n\
  file contains simply the list of pixel values with no header. \n\
  Two sets of RAW format are supported, one for the traditional \n\
  8-bits images nammed RAW8, the other for 9 to 16 bits images \n\
  named RAW16.\n");
#endif

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
INPUT SEQUENCE OPTIONS:					      Default\n\
\n\
  -p [%%s]: image or video path \n\
     . Dealing with an image sequence:                                 \n\
         By image sequence, we mean one file per image.\n\
         Specify the path and the generic name of the files \n\
         containing the images to process. The following image \n\
         file formats PNG, PNM (PGM P5, PPM P6) and RAW \n\
         (RAW8, RAW16) are supported. The format is selected by \n\
         analysing the filename extension.\n\
         Example: -p rond-point%%04d.png\n\
     . Dealing with a video:                                 \n\
         By video, we mean one file for all images of the video.\n\
         Specify the video file. Only Mpeg file are supported. \n\
         Warning: in a Mpeg2 video stream the first frame  \n\
         has the number zero. \n\
         Example: -p video.mpg -f 0 \n\
\n");
#else
  fprintf(stdout, "\n\
INPUT SEQUENCE OPTIONS:					      Default\n\
\n\
  -p [%%s]: image or video path \n\
     . Dealing with an image sequence:                                 \n\
         By image sequence, we mean one file per image.\n\
         Specify the path and the generic name of the files \n\
         containing the images to process. The following image \n\
         file formats PNM (PGM P5, PPM P6) and RAW (RAW8, RAW16)\n\
         are supported. The format is selected by analysing the \n\
         filename extension.\n\
         Example: -p rond-point%%04d.pgm\n\
     . Dealing with a video:                                 \n\
         By video, we mean one file for all images of the video.\n\
         Specify the video file. Only Mpeg file are supported. \n\
         Warning: in a Mpeg2 video stream the first frame  \n\
         has the number zero. \n\
         Example: -p video.mpg -f 0 \n\
\n");
#endif
  fprintf(stdout, "\
  -R [%%d]: Number of rows in RAW image.            \n\
     Specify the number of rows for RAW sequences.          \n\
\n\
  -C [%%d]: Number of columns in RAW image.               \n\
     Specify the number of columns for RAW sequences.          \n\
\n\
  -f [%%s]: first frame                                             1\n\
     Specify the number of the first frame in the video \n\
     sequence. If the image sequence numbering uses a fixed \n\
     number of digits, complete whith 0 before the image \n\
     number.\n\
     Warning: the first frame of a Mpeg2 video stream has the\n\
     number zero.\n\
\n\
  -s [%%d]: step                                                    1\n\
     Specify the step between two frames in the video sequence.\n\
     If step > 0 images are processed forward. If step < 0 \n\
     images are processed backward. This parameter allow the \n\
     video temporal subsampling.\n\
\n\
  -i [%%lu]: iterations                                            33\n\
     Specify the number of motion estimation iterations to\n\
     process. The number of the last image computed is given by:\n\
     first_frame + iterations * step.\n\
\n");

  fprintf(stdout, "\n\
MOTION MODEL OPTIONS (input):\n\
\n\
  -m [%%s]: model id                                               AC\n\
     Specify the parametric motion model id to estimate. The\n\
     table below gives the list of possible model_id strings: \n\
     |-----------------------------------------------------| \n\
     | model_id | type        | number of parameters       | \n\
     |-----------------------------------------------------| \n\
     | TX       | constant    |  1 (c1)                    | \n\
     | TY       | constant    |  1 (c2)                    | \n\
     | TR       | constant    |  2 (c1,c2)                 | \n\
     | ----------------------------------------------------| \n\
     | AXD      | affine      |  2 (c1,a1)                 | \n\
     | ARD      | affine      |  3 (c1,c2,a1)              | \n\
     | ARR      | affine      |  3 (c1,c2,a1)              | \n\
     | AXN      | affine      |  5 (c2,a1...a4)            | \n\
     | AYN      | affine      |  5 (c1,a1...a4)            | \n\
     | ADN      | affine      |  5 (c1,c2,a1,a2,a3         | \n\
     | ARN      | affine      |  5 (c1,c2,a1,a2,a4)        | \n\
     | AH1N     | affine      |  5 (c1,c2,a1,a2,a3)        | \n\
     | AH2N     | affine      |  5 (c1,c2,a1,a2,a4)        | \n\
     | ARRD     | affine      |  4 (c1,c2,a1,a2)           | \n\
     | AC       | affine      |  6 (c1,c2,a1...a4)         | \n\
     | ----------------------------------------------------| \n\
     | QPD      | quadratic   |  4 (c1,a1,q1,q2)           | \n\
     | QPT      | quadratic   |  4 (c1,c2,q1,q2)           | \n\
     | QPTD     | quadratic   |  5 (c1,c2,a1,q1,q2)        | \n\
     | Q2D      | quadratic   |  8 (c1,c2,a1...a4,q1,q2)   | \n\
     | QC       | quadratic   | 12 (c1,c2,a1...a4,q1...q6) | \n\
     ------------------------------------------------------| \n");
  fprintf(stdout, "\
     | MDL_TX               | same as TX                   | \n\
     | MDL_TY               | same as TX                   | \n\
     | MDL_TR               | same as TR                   | \n\
     | MDL_AFF_TX_DIV       | same as AXD                  | \n\
     | MDL_AFF_TR_DIV       | same as ARD                  | \n\
     | MDL_AFF_TR_ROT       | same as ARR                  | \n\
     | MDL_AFF_TX_NULL      | same as AXN                  | \n\
     | MDL_AFF_TY_NULL      | same as AYN                  | \n\
     | MDL_AFF_DIV_NULL     | same as ADN                  | \n\
     | MDL_AFF_ROT_NULL     | same as ARN                  | \n\
     | MDL_AFF_HYP1_NULL    | same as AH1N                 | \n\
     | MDL_AFF_HYP2_NULL    | same as AH2N                 | \n\
     | MDL_AFF_TR_ROT_DIV   | same as ARRD                 | \n\
     | MDL_AFF_COMPLET      | same as AC                   | \n\
     | MDL_QUA_PAN_DIV      | same as QPD                  | \n\
     | MDL_QUA_PAN_TILT     | same as QPT                  | \n\
     | MDL_QUA_PAN_TILT_DIV | same as QPTD                 | \n\
     | MDL_QUA_2D           | same as Q2D                  | \n\
     | MDL_QUA_COMPLET      | same as QC                   | \n\
     ------------------------------------------------------| \n\
\n\
  -g\n\
     Specify that the global illumination parameter will be\n\
     estimated.\n\
\n\
  -I\n\
     Uses the previous estimation to initialize the motion model \n\
     to estimate.\n\
\n\
  -x [%%f]: model col orig \n\
     Sets the origin (column coordinate) of the motion model.\n\
     By default, this parameter is initialized to the mid image\n\
     column number.\n\
\n\
  -y [%%f]: model row orig \n\
     Sets the origin (row coordinate) of the motion model.\n\
     By default, this parameter is initialized to the mid image\n\
     line number.\n\
\n\
  -z \n\
     Computes the covariance matrix of the motion model parameters.\n\
\n");

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
ESTIMATION SUPPORT OPTIONS (input):\n\
\n\
  -a [%%s]: support image or video path                       \n\
     Motion2D makes it possible to estimate motion either on \n\
     all the image (by defect), or on a part of the image. In \n\
     this case, it is necessary to indicate to the software the\n\
     position of the area on which the motion estimation must be \n\
     done. This is carried out while associating each image of \n\
     the video sequence a file called estimation support, corres-\n\
     ponding to an image with same size, numbering and extension \n\
     as the images to be treated. Thus, this option specify the \n\
     path and the generic name of the files containing the images \n\
     corresponding to the estimation support. The images format \n\
     depends on the filename extension. We support only Mpeg2, PNG, \n\
     PNM (PGM P5, PPM P6) and RAW (RAW8, RAW16) image file formats. \n\
     Be aware, images to proceed (those specified by option -p \n\
     and option -a) must have the same size.\n\
     For example, to estimate the motion between images \n\
     \"rond-point0001.png\" and \"rond-point0002.png\", the\n\
     support file name must be \"support0001.png\".\n");
#else
  fprintf(stdout, "\n\
ESTIMATION SUPPORT OPTIONS (input):\n\
\n\
  -a [%%s]: support image path                                  \n\
     Motion2D makes it possible to estimate motion either on \n\
     all the image (by defect), or on a part of the image. In \n\
     this case, it is necessary to indicate to the software the\n\
     position of the area on which the motion estimation must be \n\
     done. This is carried out while associating each image of \n\
     the video sequence a file called estimation support, corres-\n\
     ponding to an image with same size, numbering and extension \n\
     as the images to be treated. Thus, this option specify the \n\
     path and the generic name of the files containing the images \n\
     corresponding to the estimation support. The images format \n\
     depends on the filename extension. We support only Mpeg2, PNM \n\
     (PGM P5, PPM P6) and RAW (RAW8, RAW16) image file formats. \n\
     Be aware, images to proceed (those specified by option -p \n\
     and option -a) must have the same size.\n\
     For example, to estimate the motion between images \n\
     \"rond-point0001.pgm\" and \"rond-point0002.pgm\", the\n\
     support file name must be \"support0001.pgm\".\n");
#endif

  fprintf(stdout, "\n\
  -c [%%d]: support label \n\
     This option is associated to the previous one. It fixes the\n\
     value of the estimation support label where the estimation \n\
     will be achieved. \n\
\n");

  fprintf(stdout, "\n\
MULTI-RESOLUTION FRAMEWORK OPTIONS (input):\n\
\n\
  -q [%%d]: nbsubsample                                             0\n\
     Number of subsampling to apply to the images before\n\
     starting the motion estimation process. If this parameter\n\
     is equal to zero, images are not subsampled. If this \n\
     parameter is greater than zero, images are resized at a lower\n\
     resolution where the number of lines and columns is divided \n\
     by 2^nbsubsample.\n\
     Be aware, the estimated parametric motion model is returned \n\
     at the highest resolution.\n\
\n\
  -n [%%u]: number of pyramid levels                                4\n\
     Specify the number of levels in the Gaussian image pyramids\n\
     and image gradients pyramids used in the multi-resolution\n\
     framework. In general, when the size of the images to be\n\
     treated is close to CIF format (352 X 288), it is advised\n\
     to fix this parameter at 4. When the images are with QCIF\n\
     format (176 X 144), this parameter can be fixed at 3.\n\
     This parameter implicitly fix the index of the initial \n\
     level in the pyramids where the robust estimate begins. This \n\
     initial level is than equal to the number of levels in the \n\
     pyramids minus one. The selected level is then that of lower\n\
     resolution.\n\
\n\
  -l [%%u]: pyramid stop level                                      0\n\
      Specify the level of the image pyramids where the estimation\n\
     process will be stopped. This parameter in general fixed at\n\
     zero (corresponds then to the level of higher resolution) \n\
     can vary in the range [ 0, number_of_pyramid_levels - 1]. \n\
     The coarsest level is given by number_of_pyramid_levels - 1.  \n\
     The finest level is 0.\n\
\n");

  fprintf(stdout, "\n\
RESULTS OPTIONS (output):\n\
\n\
  -r [%%s]: dominant motion filename                        \n\
     Specify the name of the file which will contain the \n\
     estimated parametric dominant motion model results.\n\
     This result file contains the values of the estimated \n\
     2D parametric motion model given at the highest image\n\
     resolution. The parameters of the estimated motion model \n\
     are very small, especially affine and quadratic terms. To \n\
     increase these parameters resolution, a multiplier factor \n\
     can be applied to all the parameters (see option -e).\n\
\n\
  -e [%%f]: multiplier factor for the motion parameter            1.0\n\
     This option is associated with the -r option.\n\
     It defines a multiplier factor applied to all the motion \n\
     model parameter (c1,c2,a1,...a4,q1...q6) when those are \n\
     saved in a result filename specified with option -r. \n");

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
  -b [%%s]: back warped image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the back-warped images built using the estimated motion\n\
     model. We support only PNG, PNM (PGM P5, PPM P6) and RAW \n\
     (RAW8, RAW16) image file formats. The format is selected by \n\
     analysing the filename extension.\n\
     Example: -b /tmp/B%%04.png\n");
#else
  fprintf(stdout, "\n\
  -b [%%s]: back warped image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the back-warped images built using the estimated motion\n\
     model. We support only PNM (PGM P5, PPM P6) and RAW \n\
     (RAW8, RAW16) image file formats. The format is selected by \n\
     analysing the filename extension.\n\
     Example: -b /tmp/B%%04.pgm\n");
#endif

  fprintf(stdout, "\n\
  -j [%%d]: back warped image nrows                            \n\
     Set the number of rows of the back-warped image.\n\
     By default, the back-warped image has the same rows number\n\
     than the images to proceed. This option is taken into \n\
     account only when option -b is used.\n\
\n\
  -k [%%d]: back warped image ncols                            \n\
     Set the number of columns of the back-warped image.\n\
     By default, the back-warped image has the same rows number\n\
     than the images to proceed. This option is taken into \n\
     account only when option -b is used.\n\
\n");

   fprintf(stdout, "\n\
  -t [%%d]: back warped image row origin                            0\n\
     This option makes it possible to fix the co-ordinates of the \n\
     row origin of the back-warped image compared to the origin \n\
     of the images to treat. This option is taken into account \n\
     only when option -b is used.\n\
\n\
  -u [%%d]: back warped image col origin                            0\n\
     This option makes it possible to fix the co-ordinates of the \n\
     column origin of the back-warped image compared to the origin \n\
     of the images to treat. This option is taken into account \n\
     only when option -b is used.\n");

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
  -w [%%s]: weights image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the M-estimator weights. These weights in [0,1] are\n\
     rescaled in [0,255]. This map can be used to see\n\
     if a pixel participate to the robust motion estimation \n\
     (pixel in white) or is more considered as an outlier \n\
     (pixel in black). We support only PNG and PNM image file \n\
     formats. The different PNM formats are PGM (P5) and PPM (P6). \n\
     The format is selected by analysing the filename extension.\n\
     Example: -w /tmp/W%%04.png\n");
#else
  fprintf(stdout, "\n\
  -w [%%s]: weights image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the M-estimator weights. These weights in [0,1] are\n\
     rescaled in [0,255]. This map can be used to see\n\
     if a pixel participate to the robust motion estimation \n\
     (pixel in white) or is more considered as an outlier \n\
     (pixel in black). We support only PNM image file formats. \n\
     The different PNM formats are PGM (P5) and PPM (P6). \n\
     The format is selected by analysing the filename extension.\n\
     Example: -w /tmp/W%%04.pgm\n");
#endif

   fprintf(stdout, "\n\
  -F [%%s]: field vector image path                           \n\
     Specify the path and the generic name of the files contain-\n\
     ing the displacement vectors in fieldshow format. \n\
     Example: -w /tmp/F%%04.field\n\
\n\
  -o [%%s]: residual motion filename                        \n\
     Specify the name of the file which will contain the \n\
     residual motion parameters. \n");

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\n\
  -d [%%s]: residual motion image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the residual image motions. The motions in [0,8] are\n\
     rescaled in [0,255]. The generated images format depends on  \n\
     the image extension. We support only PNG, PNM (PGM P5, PPM P6) \n\
     and RAW (RAW8, RAW16) image file formats. The format is \n\
     selected by analysing the filename extension. \n\
     Example: -w /tmp/R%%04.pgm\n");
#else
  fprintf(stdout, "\n\
  -d [%%s]: residual motion image path                            \n\
     Specify the path and the generic name of the files contain-\n\
     ing the residual image motions. The motions in [0,8] are\n\
     rescaled in [0,255]. The generated images format depends on  \n\
     the image extension. We support only PNM (PGM P5, PPM P6) \n\
     and RAW (RAW8, RAW16) image file formats. The format is \n\
     selected by analysing the filename extension.\n\
     Example: -w /tmp/R%%04.pgm\n");
#endif

   fprintf(stdout, "\n\
OTHER OPTIONS:\n\
\n\
  -v\n\
     Activate the verbose mode.\n\
\n\
  -h\n\
     Print the help.\n\
\n\
  -?\n\
     Print the help.\n\
\n\
\n");

  exit(0);
}



/*!

  Print the header in the result file.

*/
void CMotion2D::printHeaderResults(FILE *output, CMotion2DModel model,
				   double multfactor)
{
    string s = model.idToString();

    fprintf(output, "# Motion2D-1.3.11 Copyright (c) 1995-2005 by INRIA\n\
# \n\
# This file contains the parameter values of the estimated 2D \n\
# parametric motion model. A comment line starts with the # \n\
# character. After the comments, the first line refers to the \n\
# estimated model id. The next line to a multiplier factor \n\
# applied to all the motion model parameters (c1,c2,a1...a4,q1..q6)\n\
# to increase their resolution. Next, each line refers to the \n\
# motion model estimated between two successive images. \n\
# \n\
# The data signification is given below. \n\
# \n\
# |--------------------------------------------------------| \n\
# | column | data signification for each estimation        | \n\
# | number | between two successive images.                | \n\
# |--------|-----------------------------------------------| \n\
# |   1    | number of the first image                     | \n\
# |--------|-----------------------------------------------| \n\
# |   2    | motion model origin (row coordinate or yc)    | \n\
# |   3    | motion model origin (column coordinate or xc) | \n\
# |--------|-----------------------------------------------| \n\
# |   4    | motion model parameter (c1)                   | \n\
# |   5    | motion model parameter (c2)                   | \n\
# |--------|-----------------------------------------------| \n\
# |   6    | motion model parameter (a1)                   | \n\
# |   7    | motion model parameter (a2)                   | \n\
# |   8    | motion model parameter (a3)                   | \n\
# |   9    | motion model parameter (a4)                   | \n\
# |--------|-----------------------------------------------| \n");

fprintf(output, "\
# |  10    | motion model parameter (q1)                   | \n\
# |  11    | motion model parameter (q2)                   | \n\
# |  12    | motion model parameter (q3)                   | \n\
# |  13    | motion model parameter (q4)                   | \n\
# |  14    | motion model parameter (q5)                   | \n\
# |  15    | motion model parameter (q6)                   | \n\
# |--------|-----------------------------------------------| \n\
# |  16    | illumination variation parameter              | \n\
# |--------|-----------------------------------------------| \n\
# |  17    | support size (only if computed, by default)   | \n\
# |--------|-----------------------------------------------| \n\
#  \n\
MODEL_ID = %s\n\
MULTIPLIER_FACTOR = %lf\n", s.c_str(), multfactor);


}
/*!
  Print the motion model parameters in the result file.

*/
void CMotion2D::printResults(FILE *output, unsigned long i1, unsigned long i2,
			     CMotion2DModel model, double support_size,
			     unsigned nbsubsample, double multfactor)
{
  double row, col;
  double parameters[CMotion2DModel::MDL_NMAX_COEF];
  double var_light;

  CMotion2DModel _model;
  // Transform the model at the highest image resolution
  _model = model.getModelLevel(-nbsubsample);
  _model.getOrigin(row, col);
  _model.getParameters(parameters);

  if (output == stdout) {
    // Print the motion estimation results
    fprintf(output, "\t2D estimated model between images %lu and %lu:\n\t",
	    i1, i2);

    fprintf(output, "origin: %f %f\n\t", row, col);

    string s = _model.idToString();
    fprintf(output, "motion model: %s\n", s.c_str());
    fprintf(output, "\tx: %f | %f %f | %f %f %f \n",
	    parameters[0], parameters[2], parameters[3],
	    parameters[6], parameters[7], parameters[8]);
    fprintf(output, "\ty: %f | %f %f | %f %f %f \n",
	    parameters[1], parameters[4], parameters[5],
	    parameters[9], parameters[10], parameters[11]);

    if (_model.getVarLight(var_light))
      fprintf(output, "\tglobal illumination: %f \n", var_light);

    if (support_size != -1.)
      fprintf(output, "\tsupport size: %f \n", support_size);
  }
  else {
    fprintf(output, "%lu %f %f ", i1, row, col);
    for (int i = 0; i < CMotion2DModel::MDL_NMAX_COEF; i++)
      fprintf(output, "%f ", parameters[i] * multfactor);
    if (!_model.getVarLight(var_light)) var_light = 0;
      fprintf(output, "%f ", var_light);
    if (support_size != -1.)
      fprintf(output, "%f ", support_size);
    fprintf(output, "\n");
  }
  fflush(output);
}



int CMotion2D::testReader(int argc, char **argv)
{
  bool ierr= true;
  getoptions(argc, argv);  // Initialize the parameters


  /**************************************************************************/
  //Modif Francois Coldefy
  /*************************************************************************/

  // initialize the reader depending on the file type: mpeg or image sequence
#ifndef __NO_IMAGEIO_MPEG_
  CMpeg2Reader mpegr;
#endif
  CImageReader imgr;

  // Set the image format by regarding the image extension
#ifndef __NO_IMAGEIO_MPEG_
  if (findChar(ipath, _mpg) || findChar(ipath, _mpeg))
    Ireader = &mpegr;
  else
#endif
    Ireader = &imgr;

  /**************************************************************************/

  Ireader->setFileName(ipath);
  Ireader->setFrameNumber(frame);

  if (!Ireader->openStream()) {
    cout <<"Can't open stream "<< ipath <<endl;
    exit(0);
  }
  Ireader->getType();
  cout << "Motion estimation in process....." << endl;

  ierr = Ireader->getFrame(I, nbsubsample);
  if (ierr == false) {
    cout <<" Error "<<endl;
    exit(-1);
  }

  int i=0;
  // Robust estimation loop
  for (long unsigned n=1; n <= niter; n++) {
    Ireader->setFrameNumber(frame);

    ierr = Ireader->getFrame(I, nbsubsample);
    if ((frame/100)*100 == frame)
      cout <<" current image "<<frame<<endl;
    i=n;
  }

  cout <<" N "<<i<<endl;
  Ireader->closeStream();
  return 0;
}

/*!

  \return The number of subsampling applied to the image. If no subsampling is
  performed, this parameter is equal to zero.

*/
unsigned CMotion2D::getNbSubsample()
{
  return nbsubsample;
}


/*!

  \return The 2D parametric motion model.

*/
CMotion2DModel &CMotion2D::getMotion2DModel()
{
  return model;
}
