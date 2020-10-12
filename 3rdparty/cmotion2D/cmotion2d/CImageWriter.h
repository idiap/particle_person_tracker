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
  \file CImageWriter.h
  \brief File to include to use CImageWriter.
*/

#ifndef CImageWriter_h
#define CImageWriter_h

#include "CWriter.h"

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

class MOTION2D_API CImageWriter: public CWriter
{
 public:
  ~CImageWriter();

  string getFileName();
  bool   writeFrame(CMotion2DImage<unsigned char> & I);
  bool   writeFrame(CMotion2DImage<short> & I);
  bool   openStream() {return true;};
  bool   closeStream() {return true; };
  void   getType() { cout <<" ImageWriter "<<endl;};
  EWriterFormat getFormat();
};

#endif
