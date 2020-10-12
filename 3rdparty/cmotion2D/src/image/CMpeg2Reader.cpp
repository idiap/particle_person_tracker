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
  \file CMpeg2Reader.cpp
  \brief Definition of the CMpeg2Reader class.
*/

/*!

  \class CMpeg2Reader

  \brief The CMpeg2Reader class implements an Mpeg2 video stream reader.

*/

#ifndef __NO_IMAGEIO_MPEG_

#include <math.h>
#include <string.h>

#include "CMpeg2Reader.h"

#define round(x)  ((x-floor(x) > 0.5 ) ? ceil(x) : floor(x))


/*!

  Mpeg2 video stream reader constructor.

*/
CMpeg2Reader::CMpeg2Reader()
{
  CReader::CReader();
  currentFrame = 0;
}

/*!

  Mpeg2 video stream  reader destructor.

*/
CMpeg2Reader::~CMpeg2Reader()
{
  codec.free();
}

/*!

  Return the video stream filename.

  \sa setFileName(), setFrameNumber()
*/
string CMpeg2Reader::getFileName()
{
  return streamName;
}
/*!

  Return the images format.

*/
CReader::EReaderFormat CMpeg2Reader::getFormat()
{

  string filename = getFileName();

  int mpeg = filename.find(_mpeg);
  int mpg  = filename.find(_mpg);

  int size = filename.size();

  if ((mpeg>0 && mpeg<size ) || (mpg>0 && mpg<size))
    return FORMAT_MPEG2;

  else {
    cerr << "Error: Only MPEG2 stream is supported..." << endl;
    return FORMAT_NOT_RECOGNIZED;
  }
}
/*!

  Extract a frame from a Mpeg2 video stream. If the video
  stream is concerned with color images, these are converted to grey
  level using the formula: \f$ I = 0.299 R + 0.587 G + 0.114 B\f$

  Depending on the \e nbsubsample parameter, the extracted frame can
  be subsampled.

  \param I The grey level image eventually subsampled.

  \param nbsubsample The number of subsampling to apply to the
  image. No subsampling is performed if this parameter is equal to
  zero.

  \param nrows Image rows number.

  \param ncols Image columns number.

  \return true if the frame was extracted, false otherwise.

*/
bool CMpeg2Reader::getFrame(CMotion2DImage<unsigned char> & I,
			    unsigned nbsubsample,
			    unsigned nrows, unsigned ncols)
{
  unsigned char * pixels;
  int ierr = true;

  while (ierr && currentFrame<frame)
    {
      ierr= codec.skipFrame();
      currentFrame++;
    }
  if (currentFrame<=frame)
    {
      ierr = codec.getFrame(&pixels);
      currentFrame++;
    }
  if (!ierr)
    return false;
  int n_cols = codec.getWidth();
  int n_rows = codec.getHeight();

  I.Init(n_rows, n_cols);
  int k=0;
  for (int i=0; i < n_rows; i++)
    for (int j=0; j < n_cols; j++)
      {
	int r = pixels[k++];
	int g = pixels[k++];
	int b = pixels[k++];
	int e = (int) round( 0.299*r + 0.587*g + 0.114*b  );

	I[i][j]=(unsigned char) (e);
      }

  for (unsigned i=0; i<nbsubsample; i++)
    I.Subsample();

  return true;
}
/*!

  Extract a color frame from a Mpeg2 video stream.

  \param rgbpixel The color image.

  \param nrows Image rows number.

  \param ncols Image columns number.

  \return true if the frame was extracted, false otherwise.

*/
bool CMpeg2Reader::getFrame(unsigned char **rgbpixel,
			    unsigned &nrows, unsigned &ncols)
{
  int ierr = true;

  while (ierr && currentFrame<frame)
    {
      ierr= codec.skipFrame();
      currentFrame++;
    }
  if (currentFrame<=frame)
    {
      ierr = codec.getFrame(rgbpixel);
      currentFrame++;
    }
  if (!ierr)
    return false;

  ncols = codec.getWidth();
  nrows = codec.getHeight();

  return true;
}
/*!

  Extract a frame from a Mpeg2 video stream. If the video
  stream is concerned with color images, these are converted to grey
  level using the formula: \f$ I = 0.299 R + 0.587 G + 0.114 B\f$

  Depending on the \e nbsubsample parameter, the extracted frame can
  be subsampled.

  \param I The grey level image eventually subsampled.

  \param nbsubsample The number of subsampling to apply to the
  image. No subsampling is performed if this parameter is equal to
  zero.
  \param nrows Image rows number.

  \param ncols Image columns number.

  \return true if the frame was extracted, false otherwise.

*/
bool CMpeg2Reader::getFrame(CMotion2DImage<short> & I,
			    unsigned nbsubsample,
			    unsigned nrows, unsigned ncols)
{
  unsigned char * pixels;
  int ierr = true;

  while (ierr && currentFrame<frame)
  {
    ierr= codec.skipFrame();
    currentFrame++;
  }
  if (currentFrame<=frame)
  {
    ierr = codec.getFrame(&pixels);
    currentFrame++;
  }
  if (!ierr)
    return false;
  int n_cols = codec.getWidth();
  int n_rows = codec.getHeight();
  this->nbsubsample = nbsubsample;

  I.Init(n_rows, n_cols);
  int k=0;
  for (int i=0; i < n_rows; i++)
    for (int j=0; j < n_cols; j++)
      {
	int r = pixels[k++];
	int g = pixels[k++];
	int b = pixels[k++];
	int e = (short) round( 0.299*r + 0.587*g + 0.114*b  );

	I[i][j]=(short) (e);
      }

  for (unsigned i=0; i<nbsubsample; i++)
    I.Subsample();

  return true;
}

/*!

  Open an Mpeg2 stream.

  \return true if the MPEG2 stream was openned, false otherwise.

*/
bool CMpeg2Reader::openStream( )
{
  EReaderFormat format;
  format = getFormat();
  if (format != FORMAT_MPEG2)
    return false;

  string filename = getFileName();

  int ierr;
  ierr = codec.openStream(filename.c_str());
  if (!ierr)
    return false;

      // return codec.initStream();
  ierr = codec.initStream();
  if (!ierr)
    return false;

  ierr = codec.skipFrame();
  if (!ierr)
    return false;
  return true;
}

/*!

  Close an Mpeg2 stream.

*/
bool CMpeg2Reader:: closeStream()
{
  return codec.closeStream();
}

/*!

  \return The number of subsampling applied to the image. If no subsampling is
  performed, this parameter is equal to zero.

*/
unsigned CMpeg2Reader::getNbSubsample()
{
  return nbsubsample;
}

#endif
