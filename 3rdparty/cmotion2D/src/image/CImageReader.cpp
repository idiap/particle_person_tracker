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
  \file CImageReader.cpp
  \brief Definition of the CImageReader class.
*/

/*!

  \class CImageReader

  \brief The CImageReader class implements an image reader.

  Image reading manipulations are restricted and only supported for PNG (see
  ReadPNG() function), RAW and PNM file format. We support RAW 8 bits (for
  graylevel images between 0 and 255, see ReadRAW8()), and RAW 16 bits (for
  graylevel images between 0 and 65534, see ReadRAW16()) image format. The
  different PNM formats are PGM P5 (see function ReadPGM()) and PPM P6 (see
  function ReadPPM()).  To know more about this image file format: \e man \e
  png, \e man \e pgm or \e man \e ppm.

*/

#include <cmotion2d/CImageReader.h>
#include <cstring>

using namespace std;

/*!

  Image reader destructor.

*/
CImageReader::~CImageReader()
{
}


/*!

  Compute and return the image filename.

  \sa setFileName(), setFrameNumber()
*/
string CImageReader::getFileName()
{
  char buf[FILENAME_MAX];
  sprintf(buf, streamName.c_str(), frame);
  string filename(buf);

  return filename;
}

/*!

  Return the reader image file format.

*/
CReader::EReaderFormat CImageReader::getFormat()
{

  string filename = getFileName();

  int PGM = filename.find(_PGM);
  int pgm = filename.find(_pgm);
  int PPM = filename.find(_PPM);
  int ppm = filename.find(_ppm);
  int raw8 = filename.find(_raw8);
  int RAW8 = filename.find(_RAW8);
  int raw16 = filename.find(_raw16);
  int RAW16 = filename.find(_RAW16);
#ifndef __NO_IMAGEIO_PNG_
  int PNG = filename.find(_PNG);
  int png = filename.find(_png);
#endif
  int size = filename.size();

  if ((PGM>0 && PGM<size ) || (pgm>0 && pgm<size))
    return FORMAT_PGM;
  else if ((PPM>0 && PPM<size) || ( ppm>0 && ppm<size))
    return FORMAT_PPM;
  else if ((RAW8>0 && RAW8<size) || ( raw8>0 && raw8<size))
    return FORMAT_RAW8;
  else if ((RAW16>0 && RAW16<size) || ( raw16>0 && raw16<size))
    return FORMAT_RAW16;

#ifndef __NO_IMAGEIO_PNG_
  else if ((PNG>0 && PNG<size) || (png>0 && png<size))
    return FORMAT_PNG;
#endif

  else {
#ifndef __NO_IMAGEIO_PNG_
    cerr << "Error: Only PNG, PNM (PGM P5 and PPM P6), RAW 8 bits and "
	 << endl << "RAW 16 bits image format are implemented..." << endl;
#else
    cerr << "Error: Only PNM (PGM P5 and PPM P6), RAW 8 bits and "
	 << endl << " RAW 16 bits image format are implemented..." << endl;
#endif
    return FORMAT_NOT_RECOGNIZED;
  }
}
/*!

  Read an image from a file. Only PNG, PNM (PGM P5 and PPM P6) and RAW 8 bits
  image format are implemented. The considered format depends on the filename
  extension. Color images are converted in grey level using the formula: \f$ I
  = 0.299 R + 0.587 G + 0.114 B\f$

  Depending on the \e nbsubsample parameter, the extracted frame can
  be subsampled.

  \param I The grey level image eventually subsampled.

  \param nbsubsample The number of subsampling to apply to the
  image. No subsampling is performed if this parameter is equal to
  zero.

  \param nrows Specify the number of rows for images in RAW8 format. Other
  image format are not concerned by this parameter.

  \param ncols Specify the number of columns for images in RAW8 format. Other
  image format are not concerned by this parameter.

  \return true if the frame was extracted, false otherwise.

*/
bool CImageReader::getFrame(CMotion2DImage<unsigned char> & I,
			    unsigned nbsubsample,
			    unsigned nrows, unsigned ncols)
{
  EReaderFormat format = getFormat();

  if (format == FORMAT_NOT_RECOGNIZED)
    return false;

  bool ret = false;
  string filename = getFileName();

  if (format == FORMAT_PGM)
    ret = ReadPGM(I, filename.c_str()); // Load the image PGM P5 file
  else if (format == FORMAT_PPM)
    ret =  ReadPPM(I, filename.c_str()); // Load the image PPM P6 file
  else if (format == FORMAT_RAW8) {
    // Load the RAW 8 bits image file
    ret =  ReadRAW8(I, filename.c_str(), nrows, ncols);
  }

#ifndef __NO_IMAGEIO_PNG_
  else if (format == FORMAT_PNG)
    ret = ReadPNG(I, filename.c_str()); // Load the image PNG file
#endif

  if (! ret)  {
#ifndef __NO_IMAGEIO_PNG_
    cerr << "Error: Only PNG, PNM (PGM P5 and PPM P6) and RAW 8 bits"
	 << endl << "image format are implemented..." << endl;
#else
    cerr << "Error: Only PNM (PGM P5 and PPM P6) and RAW 8 bits"
	 << endl << "image format are implemented..." << endl;
#endif
    return false;
  }

  for (unsigned i=0; i<nbsubsample; i++)
    I.Subsample();

  return true;
}



/*!

  Read an image from a file. Only PNG, PNM (PGM P5 and PPM P6), RAW 8 bits and
  RAW 16 bits image format are implemented. The considered format depends on
  the filename extension. Color images are converted in grey level using the
  formula: \f$ I = 0.299 R + 0.587 G + 0.114 B\f$

  Depending on the \e nbsubsample parameter, the extracted frame can
  be subsampled.

  \param I The grey level image eventually subsampled.

  \param nbsubsample The number of subsampling to apply to the
  image. No subsampling is performed if this parameter is equal to
  zero.

  \param nrows Specify the number of rows for images in RAW8 or RAW16
  format. Other image format are not concerned by this parameter.

  \param ncols Specify the number of columns for images in RAW8 or RAW16
  format. Other image format are not concerned by this parameter.

  \return true if the frame was extracted, false otherwise.

*/
bool CImageReader::getFrame(CMotion2DImage<short> & I,
			    unsigned nbsubsample,
			    unsigned nrows, unsigned ncols)
{
  EReaderFormat format = getFormat();

  if (format == FORMAT_NOT_RECOGNIZED)
    return false;

  bool ret = false;
  string filename = getFileName();
  this->nbsubsample = nbsubsample;

  if (format == FORMAT_PGM)
    ret = ReadPGM(I, filename.c_str()); // Load the image PGM P5 file
  else if (format == FORMAT_PPM)
    ret =  ReadPPM(I, filename.c_str()); // Load the image PPM P6 file
  else if (format == FORMAT_RAW8) {
    // Load the RAW 8 bits image file
    ret =  ReadRAW8(I, filename.c_str(), nrows, ncols);
  }
  else if (format == FORMAT_RAW16) {
    // Load the RAW 16 bits image file
    ret =  ReadRAW16(I, filename.c_str(), nrows, ncols);
  }
#ifndef __NO_IMAGEIO_PNG_
  else if (format == FORMAT_PNG)
    ret = ReadPNG(I, filename.c_str()); // Load the image PNG file
#endif

  if (! ret)  {
#ifndef __NO_IMAGEIO_PNG_
    cerr << "Error: Only PNG, PNM (PGM P5 and PPM P6), RAW 8 bits and 16 bits"
	 << endl << "image format are implemented..." << endl;
#else
    cerr << "Error: Only PNM (PGM P5 and PPM P6), RAW 8 bits and 16 bits"
	 << endl << "image format are implemented..." << endl;
#endif
    return false;
  }

  for (unsigned i=0; i<nbsubsample; i++)
    I.Subsample();

  return true;
}

/*!

  \return The number of subsampling applied to the image. If no subsampling is
  performed, this parameter is equal to zero.

*/
unsigned CImageReader::getNbSubsample()
{
  return nbsubsample;
}
