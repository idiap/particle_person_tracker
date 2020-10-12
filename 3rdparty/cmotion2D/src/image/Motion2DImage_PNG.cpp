/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef __NO_IMAGEIO_PNG_

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <CMotion2DImage.h>
#include <Motion2DImage_PNG.h>

#include <png.h>        /* libpng header; includes zlib.h and setjmp.h */

#include "readpng.h"
#include "writepng.h"

#define round(x)  ((x-floor(x) > 0.5 ) ? ceil(x) : floor(x))

#define DEBUG_LEVEL1 0

/*!
  \file Motion2DImage_PNG.cpp
  \brief Definition of PNG image format input/output functions.
*/

/*

  Read the contents of the PNG filename, allocate memory for the corresponding
  gray level image, convert the data in gray level, and set the bitmap whith
  the gray level data. That means that the image \e I is a "black and white"
  rendering of the original image in \e filename, as in a black and white
  photograph. In case of PNG color images, the gray level conversion is
  obtained by applying the quantization formula: \f$0,299 r + 0,587 g + 0,114
  b\f$.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa ReadPGM(), ReadPPM(), WritePNG()

*/
int ReadPNG(CMotion2DImage<unsigned char> &I, const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int rc ;
  ulg image_rowbytes;
  ulg h, w; // Image height and width
  int image_channels;
  uch *image_data = NULL;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadPNG: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadPNG: couldn't read file %s\n", filename);
    return false;
  }

  if ((rc = readpng_init(fd, &w, &h)) != 0) {
    switch (rc) {
    case 1:
      fprintf(stderr, "Error in ReadPNG: file is not a PNG file\n");
      break;
    case 2:
      fprintf(stderr, "Error in ReadPNG: file has bad IHDR\n");
      break;
    case 4:
      fprintf(stderr, "Error in ReadPNG: insufficient memory\n");
      break;
    default:
      fprintf(stderr, "Error in ReadPNG: unknown readpng_init() error\n");
      break;
    }
    fclose (fd);
    return false;
  }
  image_data = readpng_get_image(1, &image_channels, &image_rowbytes);
  readpng_cleanup(false);
  if (!image_data) {
    fprintf(stderr, "Error in ReadPNG:  unable to decode PNG image\n");
    fclose (fd);
    return false;
  }

  if ((h != (unsigned) I.GetRows())||( w != (unsigned) I.GetCols()))
  {
    if (I.Resize(h,w) == false)
    {
      fprintf(stderr, "Error in ReadPPM: can't allocate memory\n");
      fclose (fd);
      return false;
    };
  }

  if (image_channels==1) {
    // Gray level image
    memcpy(I.bitmap, image_data, I.GetRows()*I.GetCols() * sizeof(uch));
  }
  else {
    // Color image
    uch *pt;
    double r, g, b;
    pt = &image_data[0];
    for (unsigned i=0 ; i < I.GetRows()*I.GetCols() -1 ; i++, image_data ++) {
      r = *image_data ++;
      g = *image_data ++;
      b = *image_data;
#if 1
      I.bitmap[i] = (unsigned char)(round(0.299 * r + 0.587 * g + 0.114 * b));
#else
      I.bitmap[i] = (unsigned char)(0.299 * r + 0.587 * g + 0.114 * b);
#endif
    }
  }

  // Modif Fabien
  //free ((void *) image_data);
  readpng_cleanup(true);
  fclose (fd);

  return true;
}

/*

  Read the contents of the PNG filename, allocate memory for the corresponding
  gray level image, convert the data in gray level, and set the bitmap whith
  the gray level data. That means that the image \e I is a "black and white"
  rendering of the original image in \e filename, as in a black and white
  photograph. In case of PNG color images, the gray level conversion is
  obtained by applying the quantization formula: \f$0,299 r + 0,587 g + 0,114
  b\f$.

  \return false if an error occurs, or true otherwise.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa ReadPGM(), ReadPPM(), WritePNG()

*/
int ReadPNG(CMotion2DImage<short> &I, const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int rc ;
  ulg image_rowbytes;
  ulg h, w; // Image height and width
  int image_channels;
  uch *image_data = NULL;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadPNG: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadPNG: couldn't read file %s\n", filename);
    return false;
  }

  if ((rc = readpng_init(fd, &w, &h)) != 0) {
    switch (rc) {
    case 1:
      fprintf(stderr, "Error in ReadPNG: file is not a PNG file\n");
      break;
    case 2:
      fprintf(stderr, "Error in ReadPNG: file has bad IHDR\n");
      break;
    case 4:
      fprintf(stderr, "Error in ReadPNG: insufficient memory\n");
      break;
    default:
      fprintf(stderr, "Error in ReadPNG: unknown readpng_init() error\n");
      break;
    }
    fclose (fd);
    return false;
  }
  image_data = readpng_get_image(1, &image_channels, &image_rowbytes);
  readpng_cleanup(false);
  if (!image_data) {
    fprintf(stderr, "Error in ReadPNG:  unable to decode PNG image\n");
    fclose (fd);
    return false;
  }

  if ((h != (unsigned) I.GetRows())||( w != (unsigned) I.GetCols()))
  {
    if (I.Resize(h,w) == false)
    {
      fprintf(stderr, "Error in ReadPPM: can't allocate memory\n");
      fclose (fd);
      return false;
    };
  }

  if (image_channels==1) {
    // Gray level image
    for (unsigned i=0 ; i < I.GetRows()*I.GetCols() ; i++) {
      I.bitmap[i] = (short) image_data[i];
    }
  }
  else {
    // Color image
    uch *pt;
    double r, g, b;
    pt = &image_data[0];
    for (unsigned i=0 ; i < I.GetRows()*I.GetCols() -1 ; i++, image_data ++) {
      r = *image_data ++;
      g = *image_data ++;
      b = *image_data;
#if 1
      I.bitmap[i] = (short)(round(0.299 * r + 0.587 * g + 0.114 * b));
#else
      I.bitmap[i] = (short)(0.299 * r + 0.587 * g + 0.114 * b);
#endif
    }
  }

  // Modif Fabien
  //free ((void *) image_data);
  readpng_cleanup(true);
  fclose (fd);

  return true;
}


/*

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \warning The resulted PNG image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPNG(), WritePGM(), WritePPM()

*/
int WritePNG(CMotion2DImage<unsigned char> &I, const char *filename)
{
  FILE* fd;
  int rc = 0;
  int error = 0;
  mainprog_info png ;
  ulg rowbytes;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WritePNG: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WritePNG: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  png.infile = NULL;
  png.outfile = fd;
  png.image_data = NULL;
  png.row_pointers = NULL;
  png.filter = FALSE;
  png.interlaced = FALSE;
  png.have_bg = FALSE;
  png.have_time = FALSE;
  png.have_text = 0;
  png.gamma = 0.0;
  png.sample_depth = 8;  /* <==> maxval 255 */

  png.pnmtype = 5 ;
  //  ptr.pnmtype =  PNG_COLOR_TYPE_GRAY ;
  png.width = I.GetCols() ;
  png.height = I.GetRows() ;

  if ((rc = writepng_init(&png)) != 0) {
    switch (rc) {
    case 2:
      fprintf(stderr, "Error in WritePNG: libpng initialization problem\n");
      break;
    case 4:
      fprintf(stderr, "Error in WritePNG: insufficient memory\n");
      break;
    case 11:
      fprintf(stderr, "Error in WritePNG: internal logic error (unexpected PNM type)\n");
      break;
    default:
      fprintf(stderr, "Error in WritePNG: unknown writepng_init() error\n");
      break;
    }
    return rc;
  }

  rowbytes = png.width;

  {
    long j;

    /*    png.image_data = (uch *)malloc(rowbytes);
    if (png.image_data == NULL) {
      fprintf(stderr,  "Error in WritePNG: insufficient memory for row data\n");
      writepng_cleanup(&png);
      wpng_cleanup(&png);
      fclose(fd);
      return false;
      }*/
    error = 0;
    for (j = 0 ; j < png.height;  j++) {
      png.image_data = I.row[j] ;
      if (writepng_encode_row(&png) != 0) {
	fprintf(stderr, "Error in WritePNG: libpng problem (longjmp) while writing row %ld\n",
		png.height-j);
	++error;
	break;
      }
    }
    if (error) {
      writepng_cleanup(&png);
      wpng_cleanup(&png);
      fclose(fd);
      return false;
    }
    if (writepng_encode_finish(&png) != 0) {
      fprintf(stderr, "Error in WritePNG:  error on final libpng call\n");
      writepng_cleanup(&png);
      wpng_cleanup(&png);
      fclose(fd);
      return false;
    }
  }

  fflush(fd);
  writepng_cleanup(&png);
  fclose(fd);

  return true;
}
/*

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \warning The resulted PNG image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPNG(), WritePGM(), WritePPM()

*/
int WritePNG(CMotion2DImage<short> &I, const char *filename)
{
  CMotion2DImage<unsigned char> Iuchar ;
  Iuchar.Init(I.GetRows(), I.GetCols()) ;

  for (unsigned i=0 ; i < I.GetRows() * I.GetCols() ; i++)
    Iuchar.bitmap[i] =  (unsigned char) I.bitmap[i] ;

  return (WritePNG(Iuchar, filename));

}
#endif
