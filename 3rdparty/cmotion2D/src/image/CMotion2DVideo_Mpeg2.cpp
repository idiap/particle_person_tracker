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
  \file CMotion2DVideo_Mpeg2.cpp
  \brief Definition of the CMotion2DVideo_Mpeg2 class.
*/

/*!

  \class CMotion2DVideo_Mpeg2

  \brief The CMotion2DVideo_Mpeg2 class implements an Mpeg2 decoder.

*/

#ifndef __NO_IMAGEIO_MPEG_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string>

#include "config.h"
#include "global.h"
#include "mpeg2dec.h"

#include "CMotion2DVideo_Mpeg2.h"

/*!

  Mpeg2 decoder constructor.

*/
CMotion2DVideo_Mpeg2::CMotion2DVideo_Mpeg2()
{
  firstframeisread = false;
  frameNumber = 0;
  im = 0;
  width = -1;
  height = -1;
  channels = -1;
}


/*!

  Mpeg2 decoder destructor.

*/
CMotion2DVideo_Mpeg2::~CMotion2DVideo_Mpeg2()
{
  free();
}

/*!

  Open an Mpeg2 video stream.

  \return true if the Mpeg2 file was openned, false otherwise.

*/
int CMotion2DVideo_Mpeg2::openStream(const char *filename)
{
  free();
  channels =3;
  int r = OpenMPEG(filename, &height, &width);
  if (r == false) {
    return false;
  }
  if (im != 0)
    delete [] im;
  im = new unsigned char [height*width*channels];
  return r;
}

/*!

  Close an Mpeg2 video stream.

*/
int CMotion2DVideo_Mpeg2::closeStream()
{
  free();
  return CloseMPEG();
}

/*!

  Initialize the Mpeg2 video decoder.

*/
int CMotion2DVideo_Mpeg2::initStream(int getdct )
{
 return  InitMPEG( getdct);
}

/*!

  Get a decoded frame from the Mpeg2 video stream.

  \param bitmap The address of the frame.

*/
int CMotion2DVideo_Mpeg2::getFrame(unsigned char **bitmap)
{
  int r = -1;

  if (im==0 && width>0 && height >0 )
    im = new unsigned char [height*width*channels];

  if (im !=0)
    {
      r=GetMPEGFrame(im);
      (*bitmap) = im;

      if (r>0)	{
	if (firstframeisread == false)
	  firstframeisread = true;
	else
	  frameNumber++;
      }
    }

  return r;
}

/*!

  Get the current frame number in the Mpeg2 video stream.

*/
unsigned long CMotion2DVideo_Mpeg2::getFrameNumber()
{
  return frameNumber;
}

/*!

  Skip the next frame in the Mpeg2 video stream.

*/
int CMotion2DVideo_Mpeg2::skipFrame()
{
  if (firstframeisread == false)
    firstframeisread = true;
  else
    frameNumber++;

  return GetMPEGFrame(im,1);
}

/*!

  Free allocated memory for the decoder.

*/
void CMotion2DVideo_Mpeg2::free()
{
  if (im!=0)
    delete [] im;
  width = height = -1;
  im=0;
}

/*!

  Return the decoded image width.

*/
int CMotion2DVideo_Mpeg2::getWidth()
{
  return width;
}


/*!

  Return the decoded image height.

*/
int CMotion2DVideo_Mpeg2::getHeight()
{
  return height;
}

/*!

  Return the number of channels of the images; one channel for grey
  level images, and tree channels for color images.

*/
int CMotion2DVideo_Mpeg2::getChannels()
{
  return channels;
}

#endif
