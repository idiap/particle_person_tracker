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
#include <cmotion2d/Motion2DImage_PNM.h>

#define round(x)  ((x-floor(x) > 0.5 ) ? ceil(x) : floor(x))

#define MAX_LEN 100
#define DEBUG_LEVEL1 0


/*!

  \file Motion2DImage_PNM.cpp
  \brief Definition PNM image format input/output functions.
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
bool
ReadPGM(CMotion2DImage<unsigned char> &I, const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int   ierr;
  int   line;
  int   is255;
  char* cerr;
  char  str[MAX_LEN];
  unsigned   w, h;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadPGM: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL)
  {
    fprintf(stderr, "Error in ReadPGM: couldn't read file %s\n", filename);
    return false;
  }

  // Read the first line with magic number P5
  line = 0;

  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPGM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (strlen(str) != 3) {
    fprintf(stderr, "Error in ReadPGM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  str[2] = '\0';
  if (strcmp(str, "P5") != 0) {
    fprintf(stderr, "Error in ReadPGM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  // Jump the possible comment, or empty line and read the following line
  do {
    cerr = fgets(str, MAX_LEN - 1, fd);
    line++;
    if (cerr == NULL) {
      fprintf(stderr, "Error in ReadPGM: couldn't read line %d of file %s\n",
	      line, filename);
      fclose (fd);
      return false;
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract image size
  ierr = sscanf(str, "%d %d", &w, &h);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPGM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if ((h != I.GetRows())||( w != I.GetCols()))
  {
    if (I.Resize(h,w) == false)
    {
      fprintf(stderr, "Error in ReadPGM: can't allocate memory\n");
      fclose (fd);
      return false;
    };
  }

  // Read 255
  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPGM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  ierr = sscanf(str, "%d", &is255);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPGM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (is255 != 255) {
    fprintf(stderr, "Error in ReadPGM: MAX_VAL is not 255 in file %s\n",
	    filename);
    fclose (fd);
    return false;
  }

  unsigned int nbyte = I.GetRows()*I.GetCols();
  if (fread (I.bitmap, sizeof(unsigned char), nbyte, fd ) != nbyte)
  {
    fprintf(stderr, "Error in ReadPGM: couldn't read %d bytes in file %s\n",
	    nbyte, filename);
    fclose (fd);
    return false;
  }

  if (DEBUG_LEVEL1) {
    int i = 1000;
    if (i == 1000) {
      cout << "I[" << i << "]=" << (int) I.bitmap[i] << endl;
    }
  }
  fclose (fd);

  return true ;
}

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
bool
ReadPGM(CMotion2DImage<short> &I, const char *filename)
{
  CMotion2DImage<unsigned char> Iuchar ;

  if (ReadPGM(Iuchar, filename) == false)
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
bool
ReadPPM(CMotion2DImage<unsigned char> &I, const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int   ierr;
  int   line;
  int   is255;
  char* cerr;
  char  str[MAX_LEN];
  unsigned   w, h;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadPPM: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read file %s\n", filename);
    return false;
  }

  // Read the first line with magic number P6
  line = 0;

  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (strlen(str) != 3) {
    fprintf(stderr, "Error in ReadPPM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  str[2] = '\0';
  if (strcmp(str, "P6") != 0) {
    fprintf(stderr, "Error in ReadPPM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  // Jump the possible comment, or empty line and read the following line
  do {
    cerr = fgets(str, MAX_LEN - 1, fd);
    line++;
    if (cerr == NULL) {
      fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	      line, filename);
      fclose (fd);
      return false;
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract image size
  ierr = sscanf(str, "%d %d", &w, &h);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPPM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if ((h != I.GetRows())||( w != I.GetCols()))
  {
    if (I.Resize(h,w) == false)
    {
      fprintf(stderr, "Error in ReadPPM: can't allocate memory\n");
      fclose (fd);
      return false;
    };
  }

  // Read 255
  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  ierr = sscanf(str, "%d", &is255);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPPM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (is255 != 255) {
    fprintf(stderr, "Error in ReadPPM: MAX_VAL is not 255 in file %s\n",
	    filename);
    fclose (fd);
    return false;
  }

  for(unsigned i=0; i < I.GetRows(); i++)
  {
    for(unsigned j=0; j < I.GetCols(); j++)
    {
      unsigned char R, G, B;
      int res;
      res  = fread(&R, sizeof(unsigned char), 1, fd);
      res &= fread(&G, sizeof(unsigned char), 1, fd);
      res &= fread(&B, sizeof(unsigned char), 1, fd);
      if (res == 0)
      {
	unsigned nbyte = I.GetRows()*I.GetCols();
	fprintf(stderr,
		"Error in ReadPPM: couldn't read %d bytes in file %s\n",
		nbyte, filename);
	fclose (fd);
	return false;
      }
#if 1
      I[i][j] = (unsigned char)(round(0.299 * R + 0.587 * G + 0.114 * B));
#else
      I[i][j] = (unsigned char)(0.299 * R + 0.587 * G + 0.114 * B);
#endif
      if (DEBUG_LEVEL1) {
	int k = i * I.GetCols() + j;
	if (k == 1000) {
	  cout << "r=" << (int) R << " g=" << (int) G << " b=" << (int) B << endl;
	  cout << "I[" << k << "]=" << (int) I.bitmap[k] << endl;
	}
      }
    }
  }

  fclose (fd);

  return true ;
}

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
bool
ReadPPM(CMotion2DImage<short> &I, const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int   ierr;
  int   line;
  int   is255;
  char* cerr;
  char  str[MAX_LEN];
  unsigned   w, h;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in ReadPPM: no filename\n");
    return false;
  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read file %s\n", filename);
    return false;
  }

  // Read the first line with magic number P6
  line = 0;

  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (strlen(str) != 3) {
    fprintf(stderr, "Error in ReadPPM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  str[2] = '\0';
  if (strcmp(str, "P6") != 0) {
    fprintf(stderr, "Error in ReadPPM: %s is not a PGM file\n", filename);
    fclose (fd);
    return false;
  }

  // Jump the possible comment, or empty line and read the following line
  do {
    cerr = fgets(str, MAX_LEN - 1, fd);
    line++;
    if (cerr == NULL) {
      fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	      line, filename);
      fclose (fd);
      return false;
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract image size
  ierr = sscanf(str, "%d %d", &w, &h);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPPM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if ((h != I.GetRows())||( w != I.GetCols()))
  {
    if (I.Resize(h,w) == false)
    {
      fprintf(stderr, "Error in ReadPPM: can't allocate memory\n");
      fclose (fd);
      return false;
    };
  }

  // Read 255
  cerr = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (cerr == NULL) {
    fprintf(stderr, "Error in ReadPPM: couldn't read line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  ierr = sscanf(str, "%d", &is255);
  if (ierr == EOF) {
    fprintf(stderr, "Error in ReadPPM: premature EOF on line %d of file %s\n",
	    line, filename);
    fclose (fd);
    return false;
  }

  if (is255 != 255) {
    fprintf(stderr, "Error in ReadPPM: MAX_VAL is not 255 in file %s\n",
	    filename);
    fclose (fd);
    return false;
  }

  for(unsigned i=0; i < I.GetRows(); i++)
  {
    for(unsigned j=0; j < I.GetCols(); j++)
    {
      unsigned char R, G, B;
      int res;
      res  = fread(&R, sizeof(unsigned char), 1, fd);
      res &= fread(&G, sizeof(unsigned char), 1, fd);
      res &= fread(&B, sizeof(unsigned char), 1, fd);
      if (res == 0)
      {
	unsigned nbyte = I.GetRows()*I.GetCols();
	fprintf(stderr,
		"Error in ReadPPM: couldn't read %d bytes in file %s\n",
		nbyte, filename);
	fclose (fd);
	return false;
      }
#if 1
      I[i][j] = (short)(round(0.299 * R + 0.587 * G + 0.114 * B));
#else
      I[i][j] = (short)(0.299 * R + 0.587 * G + 0.114 * B );
#endif
      if (DEBUG_LEVEL1) {
	int k = i * I.GetCols() + j;
	if (k == 1000) {
	  cout << "r=" << (int) R << " g=" << (int) G << " b=" << (int) B << endl;
	  cout << "I[" << k << "]=" << (int) I.bitmap[k] << endl;
	}
      }
    }
  }

  fclose (fd);

  return true ;
}


/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \return false if an error occurs, or true otherwise.

  \sa ReadPGM(), WritePPM(), WritePNG()

*/
bool
WritePGM(CMotion2DImage<unsigned char> I, const char *filename)
{
  FILE* fd;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WritePGM: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WritePGM: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  // Write the head
  fprintf(fd, "P5\n");					// Magic number
  fprintf(fd, "%d %d\n", I.GetCols(), I.GetRows());	// Image size
  fprintf(fd, "255\n");					// Max level

  // Write the bitmap
  int ierr;
  unsigned nbyte = I.GetCols()*I.GetRows();

  ierr = fwrite(I.bitmap, sizeof(unsigned char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WritePGM: couldn't write %d bytes to file %s\n",
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
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \warning Cast the content of the bitmap from short values to unsigned char
  values. Be aware, data can be lost.

  \return false if an error occurs, or true otherwise.

  \sa ReadPGM(), WritePPM(), WritePNG()

*/
bool
WritePGM(CMotion2DImage<short> I, const char *filename)
{
  CMotion2DImage<unsigned char> Iuchar ;
  Iuchar.Init(I.GetRows(), I.GetCols()) ;

  for (unsigned i=0 ; i < I.GetRows() * I.GetCols() ; i++)
    Iuchar.bitmap[i] =  (unsigned char) I.bitmap[i] ;

  return (WritePGM(Iuchar, filename));
}

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable pixmap (PPM P5) file.

  \warning The resulted PPM P6 image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPPM(), WritePGM(), WritePNG()

*/
bool
WritePPM(CMotion2DImage<unsigned char> I, const char *filename)
{
  FILE* fd;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WritePPM: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL)
  {
    fprintf(stderr, "Error in WritePPM: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  // Write the head
  fprintf(fd, "P6\n");					// Magic number
  fprintf(fd, "%d %d\n", I.GetCols(), I.GetRows());	// Image size
  fprintf(fd, "255\n");					// Max level

  // Write the bitmap
  int ierr;
  unsigned npixel = I.GetCols()*I.GetRows();

  unsigned char rgb[3];
  for (unsigned i=0; i < npixel; i ++) {
    memset(rgb, I.bitmap[i], sizeof(rgb));
    ierr = fwrite(rgb, sizeof(unsigned char), 3, fd) ;
    if (ierr == ! 3) {
      fprintf(stderr,
	      "Error in WritePPM: couldn't write %d bytes to file %s\n",
	      3 * npixel, filename);
      fclose(fd);
      return false;
    }
  }

  fflush(fd);
  fclose(fd);

  return true;
}

/*!

  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable pixmap (PPM P5) file.

  \warning The resulted PPM P6 image is a gray level image.

  \return false if an error occurs, or true otherwise.

  \sa ReadPPM(), WritePGM(), WritePNG()

*/
bool
WritePPM(CMotion2DImage<short> I, const char *filename)
{
  FILE* fd;

  // Test the filename
  if (filename[0] == '\0')   {
    fprintf(stderr, "Error in WritePPM: no filename\n");
    return false;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL)
  {
    fprintf(stderr, "Error in WritePPM: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  // Write the head
  fprintf(fd, "P6\n");					// Magic number
  fprintf(fd, "%d %d\n", I.GetCols(), I.GetRows());	// Image size
  fprintf(fd, "255\n");					// Max level

  // Write the bitmap
  int ierr;
  unsigned npixel = I.GetCols()*I.GetRows();

  short rgb[3];
  for (unsigned i=0; i < npixel; i ++) {
    memset(rgb, I.bitmap[i], sizeof(rgb));
    ierr = fwrite(rgb, sizeof(short), 3, fd) ;
    if (ierr == ! 3) {
      fprintf(stderr,
	      "Error in WritePPM: couldn't write %d short to file %s\n",
	      3 * npixel, filename);
      fclose(fd);
      return false;
    }
  }

  fflush(fd);
  fclose(fd);

  return true;
}


#undef MAX_LEN
