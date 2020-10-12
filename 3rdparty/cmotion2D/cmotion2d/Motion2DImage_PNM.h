/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef Motion2DImage_PNM_h
#define Motion2DImage_PNM_h

#include <stdio.h>
#include <stdlib.h>

#include "CMotion2DImage.h"

using namespace std;

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

/*!
  \file Motion2DImage_PNM.h
  \brief Definition of PNM image format input/output functions.
*/

/*!

  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap whith the content of
  the file.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa WritePGM(), ReadPPM(), ReadPNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadPGM(CMotion2DImage<unsigned char> &I,
	     const char filename[FILENAME_MAX]);

/*!

  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap whith the content of
  the file.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa WritePGM(), ReadPPM(), ReadPNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadPGM(CMotion2DImage<short> &I,
	     const char filename[FILENAME_MAX]);

/*!

  Read the contents of the portable pixmap (PPM P6) filename, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap whith the gray level data. That means that the image \e I is a
  "black and white" rendering of the original image in \e filename, as in a
  black and white photograph. The quantization formula used is \f$0,299 r +
  0,587 g + 0,114 b\f$.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa WritePPM(), ReadPGM(), ReadPNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadPPM(CMotion2DImage<unsigned char> &I,
	     const char filename[FILENAME_MAX]);

/*!

  Read the contents of the portable pixmap (PPM P6) filename, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap whith the gray level data. That means that the image \e I is a
  "black and white" rendering of the original image in \e filename, as in a
  black and white photograph. The quantization formula used is \f$0,299 r +
  0,587 g + 0,114 b\f$.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa WritePPM(), ReadPGM(), ReadPNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadPPM(CMotion2DImage<short> &I,
	     const char filename[FILENAME_MAX]);

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \return false if an error occurs, or true otherwise.

  \sa ReadPGM(), WritePPM(), WritePNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WritePGM(CMotion2DImage<unsigned char> I, const char *filename);
/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \warning Cast the content of the bitmap from short values to unsigned char
  values. Be aware, data can be lost.

  \return false if an error occurs, or true otherwise.

  \sa ReadPGM(), WritePPM(), WritePNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WritePGM(CMotion2DImage<short> I, const char *filename);

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable pixmap (PPM P6) file.

  \warning The resulted PPM P6 image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPPM(), WritePGM(), WritePNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WritePPM(CMotion2DImage<unsigned char> I, const char *filename);

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable pixmap (PPM P6) file.

  \warning The resulted PPM P6 image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPPM(), WritePGM(), WritePNG()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WritePPM(CMotion2DImage<short> I, const char *filename);


#endif
