/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <stdio.h>
#include <stdlib.h>
#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif
#include <ctype.h>
#include <string.h>

// Includes lies a Motion2D
#include <CMotion2DImage.h>
#include <CMotion2DModel.h>
#include <CMotion2DWarping.h>
#include <CReader.h>
#include <CImageReader.h>
#ifndef __NO_IMAGEIO_MPEG_
#  include <CMpeg2Reader.h>
#endif
#include "CImageWriter.h"
#include "../src/inc/type.h"
#include "../src/compense/compense.h"

using namespace std;

#define GETOPTARGS	"f:hi:m:p:s:vw:?"
#define max(a,b) (a>b?a:b)

bool findChar(string ch, char *f)
{
  int n = ch.find(f);
  int size = ch.size();
  return (n>0 && n<size);
};

int  getoption (int argc, char** argv, char* pszValidOpts, char** ppszParam);
void getoptions(int argc, char **argv,
		string &ipath, unsigned long &frame,
		int &step, long unsigned &niter,
		string &mfile, string &wpath, bool &verbose);
void usage(char *name, char *badparam);
bool readModel(string filename, unsigned long frame, CMotion2DModel &model);

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0


/*

  Robust multiresolution estimation of parametric motion model.

*/
int main(int argc, char **argv)
{
  CMotion2DImage<unsigned char> I;	// Image
  CMotion2DImage<unsigned char> W;	// M-estimator weights image
  CMotion2DModel	model;		// Parametric motion model
  CMotion2DWarping	warping;	// Warping
  CReader		*Ireader=NULL;	// Image reader
  CImageReader		imgr;		// PNM or PNG image reader
#ifndef __NO_IMAGEIO_MPEG_
  CMpeg2Reader		mpegr;		// Mpeg2 image decoder
#endif
  CWriter		*Wwriter=NULL;	// Image writer
  CImageWriter		imgw;		// PNM or PNG image writer

  string ipath = "../../test/sequence/rond-point"; // Image path
  string filename;		// Complete filename for an image of the video
  string mfile;			// Motion model filename
  string wpath;			// Warped image path
  long unsigned niter = 33;	// Number of images to process
  int  step = 1;		// Step between 2 images
  bool verbose = false;		// Verbose mode
  bool ierr = true;		// Image IO return value
  unsigned long frame  = 1; // Current frame number to process

  // Read the program options
  getoptions(argc, argv, ipath, frame, step, niter,
	     mfile, wpath, verbose);

  // Set the image reader format by regarding the image extension
#ifndef __NO_IMAGEIO_MPEG_
  if (findChar(ipath, _mpg) || findChar(ipath, _mpeg)) {
    cout << "Reader mpeg\n";
    Ireader = &mpegr;
  }
  else {
#endif
    cout << "Reader PNM\n";
    Ireader = &imgr;
#ifndef __NO_IMAGEIO_MPEG_
  }
#endif

  Ireader->setFileName(ipath);
  Ireader->setFrameNumber(frame);
  filename = Ireader->getFileName();

  if (!Ireader->openStream()) {
    cout <<"Can't open stream "<< ipath <<endl;
    exit(-1);
  }

  // Set the image writer
  Wwriter = &imgw;

  // Start the warping loop
  long unsigned i=0;
  do {

    // Load the image
    Ireader->setFileName(ipath);
    Ireader->setFrameNumber(frame);
    filename = Ireader->getFileName();
    if (verbose)
      cout << "Load image: " << filename << endl ;
    ierr = Ireader->getFrame(I);
    if (ierr == false) {
      cout << "Can not read the image: " << filename << endl;
      exit(-1);
    }

    if (readModel(mfile, frame, model) == false)
      exit(0);

    if (verbose) {
      double row, col;
      double p[CMotion2DModel::MDL_NMAX_COEF]; // model parameters
      model.getOrigin(row, col);
      model.getParameters(p);
      cout << "Warp image: " << filename.c_str() << endl;
      cout << "\tMotion model: " << model.idToString() << endl;
      cout << "\t  origin: " << row << " " << col << endl;
      cout << "\t  x: " << p[0] <<" | " << p[2] << " " << p[3] << " | "
	   << p[6] << " " << p[7] << " " << p[8] << endl;
      cout << "\t  y: " << p[1] <<" | " << p[4] << " " << p[5] << " | "
	   << p[9] << " " << p[10] << " " << p[11] << endl;
    }

    // Initialize the warped image size
    W.Init(I.GetRows(), I.GetCols());

    // warp the input image
    warping.warp(I.bitmap, W.bitmap, I.GetRows(), I.GetCols(), model);

    // Construct the backwarped image filename
    Wwriter->setFileName(wpath);
    Wwriter->setFrameNumber(frame);
    filename = Wwriter->getFileName();
    if (verbose)
      cout << "\tWrite warped image: " << filename << endl ;
    ierr = Wwriter->writeFrame(W);    // Write the backwarped image
    if (ierr == false) {
      cout << "Can not write the backwarped image: " << filename << endl;
      exit(-1);
    }

    frame += step; // Update the image frame number

  } while (++i < niter);

  Ireader->closeStream();

  return 0;
}



/*

  Print the program options.

 */
void usage(char *name, char *badparam)
{
  if (badparam)
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);

  fprintf(stdout, "\n\
  Copyright (c) 1995-2005 by INRIA.\n\
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
  %s [-p image_path] [-e image_extension] [-f first_frame]\n\
  [-s step] [-i iterations] [-m model_filename] [-w warped_image_path]\n\
  [-v] [-h] [-?]\n\
\n\
DESCRIPTION\n\
  This software generates warped images using a parametric motion model.\n\
\n\
\n", name);

#ifndef __NO_IMAGEIO_PNG_
  fprintf(stdout, "\
INPUT SEQUENCE OPTIONS:					      Default\n\
\n\
  -p image_path [%%s]                  ../../test/sequence/rond-point\n\
     Specify the path and the generic name of the files \n\
     containing the images to process. The following image\n\
     file formats PNG and PNM are supported. The different\n\
     PNM formats are PGM (P5) and PPM (P6).\n\
\n\
  -e image_extension [%%s]                                       .pgm\n\
     Specify the image extension. Supported image formats \n\
     are PNG (use -e .png), PGM P5 (use -e .pgm) and PPM P6\n\
     (use -e .ppm). \n\
\n");
#else
  fprintf(stdout, "\
INPUT SEQUENCE OPTIONS:					      Default\n\
\n\
  -p image_path [%%s]                  ../../test/sequence/rond-point\n\
     Specify the path and the generic name of the files \n\
     containing the images to process. Only the PNM formats \n\
     PGM (P5) and PPM (P6) are supported.\n\
\n\
  -e image_extension [%%s]                                       .pgm\n\
     Specify the image extension. Supported image formats \n\
     are PGM P5 (use -e .pgm) and PPM P6 (use -e .ppm). \n\
\n");
#endif

  fprintf(stdout, "\
  -f first_frame [%%s]                                           0001\n\
     Specify the number of the first frame in the video \n\
     sequence. If the image sequence numbering uses a fixed \n\
     number of digits, complete whith 0 before the image number.\n\
\n\
  -s step [%%d]                                                     1\n\
     Specify the step between two frames in the video sequence.\n\
     If step > 0 images are processed forward. If step < 0 images\n\
     are processed backward.\n\
\n\
  -i iterations [%%lu]                                             33\n\
     Specify the number of motion estimation iterations to\n\
     process. The number of the last image computed is given by:\n\
     first_frame + iterations * step.\n\
\n\
  -m model_filename [%%s]                       \n\
     Specify the name of the file containing the values of the\n\
     parametric motion model coefficients.\n\
\n\
\n\
RESULTS OPTIONS:\n\
\n\
  -w warped_image_path [%%s]                           \n\
     Specify the path and the generic name of the files contain-\n\
     ing the back-warped images built using the estimated motion\n\
     model. The generated images format depends on the treated  \n\
     image extension specified by option -e.\n\
\n");

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

/*

  Set the program options.

*/
void getoptions(int argc, char **argv,
		string &ipath, unsigned long &frame,
		int &step, long unsigned &niter,
		string &mfile, string &wpath, bool &verbose)
{
  char *optarg;
  int	c;
  while ((c = getoption(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'f': frame = (unsigned long) atoi(optarg); break;
    case 'h': usage(argv[0], NULL); break;
    case 'i': niter = atoi(optarg); break;
    case 'm': mfile = optarg; break;
    case 'p': ipath = optarg; break;
    case 's': step = atoi(optarg); break;
    case 'v': verbose = true; break;
    case 'w': wpath = optarg; break;
    case '?': usage(argv[0], NULL); break;

    default:  usage(argv[0], NULL); break;
    }
  }

  // Some option tests
  if (wpath.empty()) {
    // standalone param or error
    fprintf(stderr, "Option -w [warped_image_path] not specified\n");
    usage(argv[0], NULL);
    exit(0);


  }
  if ((c == 1) || (c == -1)) {
    // standalone param or error
    fprintf(stderr, "Bad argument %s\n", optarg);

    usage(argv[0], NULL);

    exit(0);
  }

  if ( mfile.empty() ) {
    cout << endl << "Error: argument -m <filename> not specified " << endl;
    exit(0);
  }
  else {
    FILE *fd = fopen(mfile.c_str(), "rb");
    if (fd == NULL) {
      cout << endl
	   << "Bad argument -m: "
	   << "Cannot open the filename specified "
	   << endl
	   << "with this option."
	   << endl;
      exit(0);
    }
    fclose(fd);
  }
}

/*

  Get next command line option and parameter

  PARAMETERS:

      argc - count of command line arguments
      argv - array of command line argument strings
      pszValidOpts - string of valid, case-sensitive option characters,
                     a colon ':' following a given character means that
                     option can take a parameter
      ppszParam - pointer to a pointer to a string for output

  RETURNS:

      If valid option is found, the character value of that option
          is returned, and *ppszParam points to the parameter if given,
          or is NULL if no param
      If standalone parameter (with no option) is found, 1 is returned,
          and *ppszParam points to the standalone parameter
      If option is found, but it is not in the list of valid options,
          -1 is returned, and *ppszParam points to the invalid argument
      When end of argument list is reached, 0 is returned, and
          *ppszParam is NULL
*/
int getoption (
    int argc,
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

bool readModel(string filename, unsigned long frame, CMotion2DModel &model)
{
#define MAX_LEN 512

  FILE *fd = NULL;
  char  str[MAX_LEN];
  int   line;
  char* cerr;
  int   ierr;

  if ( filename.empty() ) {
    fprintf(stderr, "Error in readModel: no filename\n");
    return false;
  }

  fd = fopen(filename.c_str(), "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in readModel: couldn't read file %s\n",
	    filename.c_str());
    return false;
  }

  // Jump the possible comment, or empty line and read the following line
  line = 0;
  do {
    cerr = fgets(str, MAX_LEN - 1, fd);

    line++;
    if (cerr == NULL) {
      fprintf(stderr, "Error in readModel: couldn't read line %d of file %s\n",
	      line, filename.c_str());
      fclose (fd);
      return false;
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract model id
  char _text[255];
  char _egal[255];
  char _id[255];
  string s = str;
  ierr = s.find("MODEL_ID");
  if (ierr != -1) {
    ierr = sscanf(str, "%s %s %s", _text, _egal, _id);
    //    printf("text: %s\n", _text);
    //   printf("_egal: %s\n", _egal);
    //  printf("_id: %s\n", _id);
  }
  else {
     ierr = sscanf(str, "%s", _id);
  }

  if (ierr == EOF) {
    fprintf(stderr,
	    "Error in readModel: premature EOF on line %d of file %s\n",
	    line, filename.c_str());
    fclose (fd);
    return false;
  }

  unsigned long _frame;
  double _row, _col; // model origin
  double _p[CMotion2DModel::MDL_NMAX_COEF]; // model parameters
  double _multfactor = 1.0;
  bool multfactor_was_read = false;

  do {
    // Read a line
    cerr = fgets(str, MAX_LEN - 1, fd);
    //printf("line2: %s\n", str);
    line ++;
    if (cerr == NULL) {
      fprintf(stderr, "Error in readModel: couldn't read line %d of file %s\n",
	      line, filename.c_str());
      fclose (fd);
      return false;
    }

    // Get the multiplier factor if it exists
    if (multfactor_was_read == false) {
      s = str;
      ierr = s.find("MULTIPLIER_FACTOR");
      multfactor_was_read = true;
      if (ierr != -1) {
	ierr = sscanf(str, "%s %s %lf", _text, _egal, &_multfactor);
	cout << "multfactor: " << _multfactor << endl;
	if (ierr == EOF) {
	  fprintf(stderr,
		  "Error in readModel: premature EOF on line %d of file %s\n",
		  line, filename.c_str());
	  fclose (fd);
	  return false;
	}
	continue;
      }
    }

    // Extract information
    ierr = sscanf(str,
		  "%ld %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		  &_frame, &_row, &_col,
		  &_p[0], &_p[1], // c1, c2
		  &_p[2], &_p[3], &_p[4], &_p[5], //a1, a2, a3, a4
		  &_p[6], &_p[7], &_p[8], &_p[9], &_p[10], &_p[11]); //q1...q6
    if (ierr == EOF) {
      fprintf(stderr,
	      "Error in readModel: premature EOF on line %d of file %s\n",
	      line, filename.c_str());
      fclose (fd);
      return false;

    }
  } while (frame != _frame);

  // rescale the parameters
  for (int i=0; i < CMotion2DModel::MDL_NMAX_COEF; i ++) {
    _p[i] /= _multfactor;
  }

  string id = _id;

  model.reset();	// Set all the parameters to zero
  model.setIdModel(id); // Set the model id
  model.setVarLight(false); // Set the illumination variation
  model.setOrigin(_row, _col); // Set the origin
  model.setParameters(_p);

  fclose (fd);

  return true;

#undef MAX_LEN
}


