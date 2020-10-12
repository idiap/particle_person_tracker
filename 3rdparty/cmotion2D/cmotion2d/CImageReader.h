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
  \file CImageReader.h
  \brief File to include to use CImageReader.
*/

#ifndef CImageReader_h
#define CImageReader_h

#include "CReader.h"

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

class MOTION2D_API CImageReader: public CReader
{
 public:
  ~CImageReader();

  string getFileName();
  bool   getFrame(CMotion2DImage<unsigned char> & I, unsigned nbsubsample,
		    unsigned nrows=0, unsigned ncols=0);
  bool   getFrame(CMotion2DImage<short> & I, unsigned nbsubsample,
		  unsigned nrows=0, unsigned ncols=0);
  bool   openStream() {return true;};
  bool   closeStream() {return true; };
  void   getType() { cout <<" ImageReader "<<endl;};
  EReaderFormat getFormat();
  unsigned getNbSubsample();
};

#endif
