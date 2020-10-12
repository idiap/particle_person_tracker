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
  \file CMpeg2Reader.h
  \brief File to include to use CMpeg2Reader.
*/

#ifndef CMpeg2Reader_h
#define CMpeg2Reader_h

#ifndef __NO_IMAGEIO_MPEG_

#include <CReader.h>
#include <CMotion2DVideo_Mpeg2.h>

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

class MOTION2D_API CMpeg2Reader: public CReader
{
private:
  CMotion2DVideo_Mpeg2 codec;
  unsigned long currentFrame;

public:
  CMpeg2Reader();
  ~CMpeg2Reader();
  string getFileName();
  bool   getFrame(CMotion2DImage<unsigned char> & I, unsigned nbsubsample,
		  unsigned nrows=0, unsigned ncols=0);
  bool   getFrame(CMotion2DImage<short> & I, unsigned nbsubsample,
		  unsigned nrows=0, unsigned ncols=0);
  bool   getFrame(unsigned char **rgbpixel, unsigned &nrows, unsigned &ncols);
  bool   initStream();
  bool   closeStream();
  bool   openStream();
  void   getType() { cout << " Mpeg2Reader"<<endl;};
  EReaderFormat getFormat();
  unsigned getNbSubsample();
};

#endif
#endif
