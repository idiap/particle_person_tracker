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
  \file CReader.h
  \brief File to include to use CReader.
*/

#ifndef CReader_h
#define CReader_h

#include <string>
#include "CMotion2DImage.h"

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


// Supported image format
#ifndef __NO_IMAGEIO_PNG_
#  define _PNG  "PNG"
#  define _png  "png"
#endif
#define _PGM   "PGM"
#define _pgm   "pgm"
#define _PPM   "PPM"
#define _ppm   "ppm"
#define _RAW8  "RAW8"
#define _raw8  "raw8"
#define _RAW16 "RAW16"
#define _raw16 "raw16"

#ifndef __NO_IMAGEIO_MPEG_
#  define _mpeg "mpeg"
#  define _mpg  "mpg"
#endif


using namespace std;

class MOTION2D_API CReader
{
 protected:
  unsigned long frame;
  string streamName;
  unsigned nbsubsample;

 public:
  /*!

  \enum EReaderFormat The supported image or video stream reader format.

  */
  enum EReaderFormat {
    FORMAT_NOT_RECOGNIZED, /*!< Reader format not supported. */
    FORMAT_PGM,            /*!< PNM PGM P5 image format.*/
    FORMAT_PPM,            /*!< PNM PPM P6 image format.*/
    FORMAT_RAW8,           /*!< RAW8 image format. */
    FORMAT_RAW16,          /*!< RAW16 image format.*/
#ifndef __NO_IMAGEIO_PNG_
    FORMAT_PNG,            /*!< PNG image format.*/
#endif
#ifndef __NO_IMAGEIO_MPEG_
    FORMAT_MPEG2           /*!< MPEG2 image format.*/
#endif
  };

  CReader();

  /*!

    Destructor.

  */
  virtual ~CReader() { };

  void setFileName(const char *filename);
  void setFileName(string filename);
  bool setFrameNumber(unsigned long framenumber);
  /*!
    Return the considered image or video stream reader format.

    \sa EReaderFormat
  */
  virtual EReaderFormat getFormat() {return FORMAT_NOT_RECOGNIZED;};

  /*!

    Return the filename of the image or video stream.

  */
  virtual string getFileName() {return "No filename";};

  /*!

    Read a frame from an image or video stream.

  */
  virtual bool getFrame(CMotion2DImage<unsigned char> & I,
			unsigned nbsubsample=0,
			unsigned nrows=0, unsigned ncols=0)
  {
    cout <<" Reader "<<endl;
    return false;
  };

  /*!

    Read a frame from an image or video stream.

  */
  virtual bool getFrame(CMotion2DImage<short> & I,
			unsigned nbsubsample=0,
			unsigned nrows=0, unsigned ncols=0)
  {
    cout <<" Reader "<<endl;
    return false;
  };

  /*!

    Open an image or video stream.

  */
  virtual bool openStream() {return true;};
  /*!

    Close an image or video stream.

  */
  virtual bool closeStream() {return true;};
  /*!

    Print the type of the reader.

  */
  virtual void getType() {
    cout << " Reader"<<endl;
  };

  /*!

    \return The number of subsampling applied to the image. If no subsampling
    is performed, this parameter is equal to zero.

  */
  virtual unsigned getNbSubsample() {return 0;};
};


#endif
