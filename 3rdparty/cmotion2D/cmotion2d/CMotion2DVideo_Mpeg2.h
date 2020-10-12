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
  \file CMotion2DVideo_Mpeg2.h
  \brief File to include to use CMotion2DVideo_Mpeg2.
*/

#ifndef CMotion2DVideo_Mpeg2_h
#define CMotion2DVideo_Mpeg2_h

#ifndef __NO_IMAGEIO_MPEG_

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

class MOTION2D_API CMotion2DVideo_Mpeg2
{
 private:
  unsigned char * im;
  int width, height, channels;
  unsigned long frameNumber;
  bool firstframeisread;

 public:
  CMotion2DVideo_Mpeg2();
  ~CMotion2DVideo_Mpeg2();
  int getWidth();
  int getHeight();
  int getChannels(); // number if colors (set to 3)

  int openStream(const char *filename);
  int initStream(int i=0);
  int closeStream();

  int getFrame(unsigned char **bitmap);
  unsigned long getFrameNumber();
  int skipFrame();

  void free();      // field destructor
};



#endif
#endif
