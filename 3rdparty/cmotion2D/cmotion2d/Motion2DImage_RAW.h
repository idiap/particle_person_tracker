/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef Motion2DImage_RAW_h
#define Motion2DImage_RAW_h

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
  \file Motion2DImage_RAW.h
  \brief Definition of RAW image format input/output functions.

  RAW file format images are simply made of the list of pixel values with no
  header. That is why the number of columns and of rows must be given by the
  user.

  Two sets of functions exist, one for the traditional 8-bits images named RAW8
  (see functions ReadRAW8() and WriteRAW8()), the other for 9 to 16 bits images
  named RAW16 (see functions ReadRAW16() and WriteRAW16()).

*/

/*!

  Read the contents of the purely raw images (without header), allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap with the gray level data.
  It process 8-bits data (graylevel between 0 and 255).

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return false if an error occurs, or true otherwise.

  \sa ReadRAW16(), WriteRAW8(), WriteRAW16()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadRAW8(CMotion2DImage<unsigned char> &I,
	      const char filename[FILENAME_MAX],
	      unsigned nb_rows, unsigned nb_cols);
/*!

  Read the contents of the purely raw images (without header), allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap with the gray level data.
  It process 8-bits data (graylevel between 0 and 255).

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return false if an error occurs, or true otherwise.

  \sa ReadRAW16(), WriteRAW8(), WriteRAW16()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadRAW8(CMotion2DImage<short> &I,
	      const char filename[FILENAME_MAX],
	      unsigned nb_rows, unsigned nb_cols);
/*!

  Read the contents of the purely raw images (without header), allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap with the gray level data.
  It process 16 bits data (level between 0 and 65534)
  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa ReadRAW8(), WriteRAW8(), WriteRAW16()

  \return false if an error occurs, or true otherwise.

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool ReadRAW16(CMotion2DImage<short> &I,
	       const char filename[FILENAME_MAX],
	       unsigned nb_rows, unsigned nb_cols);

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable RAW pixmap ("just a list of
  grayvalues") file without header.

  It process 8-bits data (graylevel between 0 and 255).

  \return false if an error occurs, or true otherwise.

  \sa WriteRAW16(), ReadRAW8(), ReadRAW16()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WriteRAW8(CMotion2DImage<unsigned char> I, const char *filename);
/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable RAW pixmap ("just a list of
  grayvalues") file without header.

  It process 8-bits data (graylevel between 0 and 255).

  \warning Cast the content of the bitmap from short values to unsigned char
  values. Be aware, data can be lost.

  \return false if an error occurs, or true otherwise.

  \sa WriteRAW16(), ReadRAW8(), ReadRAW16()

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WriteRAW8(CMotion2DImage<short> I, const char *filename);
/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable RAW pixmap ("just a list of
  grayvalues") file without header.

  It process 16 bits data (level between 0 and 65534).

  \return false if an error occurs, or true otherwise.

  \sa WriteRAW8(), ReadRAW8(), ReadRAW16(),

*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
MOTION2D_API
#endif
bool WriteRAW16(CMotion2DImage<short> I, const char *filename);

#endif
