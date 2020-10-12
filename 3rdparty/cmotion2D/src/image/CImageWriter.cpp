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
  \file CImageWriter.cpp
  \brief Definition of the CImageWriter class.
*/

/*!

  \class CImageWriter

  \brief The CImageWriter class implements an image writer.

  Image writing manipulations
  are restricted and only supported for PNG (see WritePNG()
  function) and PNM file format. The different PNM formats are PGM P5 (see
  function WritePGM()) and PPM P6 (see function WritePPM()).
  To know more about this image file format: \e man \e png, \e man
  \e pgm or \e man \e ppm.

*/

#include <cmotion2d/CImageWriter.h>
#include <cstring>

using namespace std;

/*!

  Image writer destructor.

*/
CImageWriter::~CImageWriter()
{
}

/*!

  Compute and return the image filename.

  \sa setFileName(), setFrameNumber()
*/
string CImageWriter::getFileName()
{
  char buf[FILENAME_MAX];
  sprintf(buf, streamName.c_str(), frame);
  string filename(buf);

  return filename;
}

/*!

  Return the reader image file format.

*/
CWriter::EWriterFormat CImageWriter::getFormat()
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

  Save an image in a file. Only PNG and PNM (PGM P5 and PPM P6)
  image format are implemented. The considered format depends on the
  filename extension. We consider only grey level images even if PNG or
  PPM P6 format support color images.

  \param I The grey level image to save.
  \return true if the frame was saved, false otherwise.

*/
bool CImageWriter::writeFrame(CMotion2DImage<unsigned char> & I)
{
  EWriterFormat format = getFormat();
  if (format == FORMAT_NOT_RECOGNIZED)
    return false;

  bool ret = false;
  string filename = getFileName();

  if (format == FORMAT_PGM)
    ret = WritePGM(I, filename.c_str()); // Write the image PGM P5 file
  else if (format == FORMAT_PPM)
    ret = WritePPM(I, filename.c_str()); // Write the image PPM P6 file
  else if (format == FORMAT_RAW8)
    ret =  WriteRAW8(I, filename.c_str()); // Write the RAW 8 bits image file

#ifndef __NO_IMAGEIO_PNG_
  else if (format == FORMAT_PNG)
    ret = WritePNG(I, filename.c_str()); // Write the image PNG file
#endif

  if (! ret)  {
#ifndef __NO_IMAGEIO_PNG_
    cerr << "Error: Only PNG and PNM (PGM P5 and PPM P6) and RAW 8 bits"
	 << endl << "image format are implemented..." << endl;
#else
    cerr << "Error: Only PNM (PGM P5 and PPM P6) and RAW 8 bits"
	 << endl << "image format are implemented..." << endl;
#endif
    return false;
  }

  return true;
}

/*!

  Save an image in a file. Only PNG and PNM (PGM P5 and PPM P6)
  image format are implemented. The considered format depends on the
  filename extension. We consider only grey level images even if PNG or
  PPM P6 format support color images.

  \param I The grey level image to save.
  \return true if the frame was saved, false otherwise.

*/
bool CImageWriter::writeFrame(CMotion2DImage<short> & I)
{
  EWriterFormat format = getFormat();
  if (format == FORMAT_NOT_RECOGNIZED)
    return false;

  bool ret = false;
  string filename = getFileName();

  if (format == FORMAT_PGM)
    ret = WritePGM(I, filename.c_str()); // Write the image PGM P5 file
  else if (format == FORMAT_PPM)
    ret = WritePPM(I, filename.c_str()); // Write the image PPM P6 file
  else if (format == FORMAT_RAW8)
    ret =  WriteRAW8(I, filename.c_str()); // Write the RAW 8 bits image file
  else if (format == FORMAT_RAW16)
    ret =  WriteRAW8(I, filename.c_str()); // Write the RAW 16 bits image file

#ifndef __NO_IMAGEIO_PNG_
  else if (format == FORMAT_PNG)
    ret = WritePNG(I, filename.c_str()); // Write the image PNG file
#endif


  if (! ret)  {
#ifndef __NO_IMAGEIO_PNG_
    cerr << "Error: Only PNG and PNM (PGM P5 and PPM P6), RAW 8 bits and "
	 << endl << "16 bits image format are implemented..." << endl;
#else
    cerr << "Error: Only PNM (PGM P5 and PPM P6), RAW 8 bits and 16 bits "
	 << endl << "image format are implemented..." << endl;
#endif
    return false;
  }

  return true;
}



