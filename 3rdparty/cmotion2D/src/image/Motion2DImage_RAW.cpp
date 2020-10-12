/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include <cmotion2d/CMotion2DImage.h>
#include <cmotion2d/Motion2DImage_RAW.h>


#define MAX_LEN 100
#define DEBUG_LEVEL1 0


/*!
  \file Motion2DImage_RAW.cpp

  \brief Definition RAW image format input/output functions.

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
  set the bitmap whith the gray level data.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa ReadRAW16(), WriteRAW8(), WriteRAW16()

  \return false if an error occurs, or true otherwise.

*/
bool
ReadRAW8(CMotion2DImage<unsigned char> &I, const char *filename,
	 unsigned nb_rows, unsigned nb_cols)
{
  FILE* fd = NULL; // File descriptor

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadRAW: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadRAW: couldn't read file %s\n", filename);
    return false;
  }

  // Test image size
  if ((nb_rows != I.GetRows())||( nb_cols != I.GetCols()))
  {
    if (I.Resize(nb_rows, nb_cols) == false)
    {
      fprintf(stderr, "Error in ReadRAW: can't allocate memory\n");
      fclose (fd);
      return false;
    }
  }

  unsigned int nbyte = nb_rows * nb_cols;
  if (fread (I.bitmap, sizeof(unsigned char), nbyte, fd ) != nbyte)
  {
    fprintf(stderr, "Error in ReadRaw8: couldn't read %d bytes in file %s\n",
	    nbyte, filename);
    fclose (fd);
    return false;
  }

  fclose (fd);

  return true ;
}

/*!

  Read the contents of the purely raw images, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap whith the gray level data.
  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return false if an error occurs, or true otherwise.

  \sa ReadRAW16(), WriteRAW8(), WriteRAW16()

*/
bool
ReadRAW8(CMotion2DImage<short> &I, const char *filename,
	 unsigned nb_rows, unsigned nb_cols)
{

  CMotion2DImage<unsigned char> Iuchar ;

  if (ReadRAW8(Iuchar, filename, nb_rows, nb_cols) == false)
    return false;

  unsigned h = Iuchar.GetRows();
  unsigned w = Iuchar.GetCols();
  unsigned size = h * w;

  if ((h != I.GetRows())||(w != I.GetCols()))
  {
    if (I.Resize(h, w) == false)
    {
      fprintf(stderr, "Error in ReadPGM: can't allocate memory\n");
      return false;
    };
  }

  for(unsigned i=0; i < size; i ++) {
    I.bitmap[i] = (short) Iuchar.bitmap[i];
  }

  return true;
}

/*!

  Read the contents of the purely raw images (without header), allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap with the gray level data.
  It process 16 bits data (level between 0 and 65534)
  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return false if an error occurs, or true otherwise.

  \sa ReadRAW8(), WriteRAW8(), WriteRAW16()

*/
bool
ReadRAW16(CMotion2DImage<short> &I, const char *filename,
	  unsigned nb_rows, unsigned nb_cols)
{
  FILE* fd = NULL; // File descriptor

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadRAW: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadRAW: couldn't read file %s\n", filename);
    return false;
  }

  if ((nb_rows != I.GetRows())||( nb_cols != I.GetCols()))
  {
    if (I.Resize(nb_rows, nb_cols) == false)
    {
      fprintf(stderr, "Error in ReadRAW: can't allocate memory\n");
      fclose (fd);
      return false;
    }
  }

  unsigned int nbyte = I.GetRows()*I.GetCols();
  if (fread (I.bitmap, sizeof(short), nbyte, fd ) != nbyte)
  {
    fprintf(stderr, "Error in ReadRaw8: couldn't read %d short in file %s\n",
	    nbyte, filename);
    fclose (fd);
    return false;
  }

  fclose (fd);

  return true ;
}



/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable RAW pixmap ("just a list of
  grayvalues") file without header.

  It process 8-bits data (graylevel between 0 and 255).

  \return false if an error occurs, or true otherwise.

  \sa WriteRAW16(), ReadRAW8(), ReadRAW16()

*/
bool
WriteRAW8(CMotion2DImage<unsigned char> I, const char *filename)
{
  FILE* fd;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WriteRAW8: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WriteRAW8: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  // Write the bitmap
  int ierr;
  unsigned nbyte = I.GetCols()*I.GetRows();

  ierr = fwrite(I.bitmap, sizeof(unsigned char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WriteRAW8: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    return false;
  }

  fflush(fd);
  fclose(fd);

  return true;
}

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
bool
WriteRAW8(CMotion2DImage<short> I, const char *filename)
{
  CMotion2DImage<unsigned char> Iuchar ;
  Iuchar.Init(I.GetRows(), I.GetCols()) ;

  for (unsigned i=0 ; i < I.GetRows() * I.GetCols() ; i++)
    Iuchar.bitmap[i] =  (unsigned char) I.bitmap[i] ;

  return (WriteRAW8(Iuchar, filename));
}

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable RAW pixmap ("just a list of
  grayvalues") file without header.

  It process 16 bits data (level between 0 and 65534).

  \return false if an error occurs, or true otherwise.

  \sa WriteRAW8(), ReadRAW8(), ReadRAW16(),

*/
bool
WriteRAW16(CMotion2DImage<short> I, const char *filename)
{
  FILE* fd;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WriteRAW16: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WriteRAW16: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  // Write the bitmap
  int ierr;
  unsigned nbyte = I.GetCols()*I.GetRows();

  ierr = fwrite(I.bitmap, sizeof(short), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WriteRAW16: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    return false;
  }

  fflush(fd);
  fclose(fd);

  return true;
}


#undef MAX_LEN
