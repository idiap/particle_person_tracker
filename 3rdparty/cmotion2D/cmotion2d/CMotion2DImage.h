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
  \file CMotion2DImage.h
  \brief File to include to use CMotion2DImage.
*/

/*!

  \class CMotion2DImage

  \brief The CMotion2DImage class provides basic image manipulations.

  <h3> Data structure </h3>

  Each image is build using two structures (an array bitmap which size
  is [nb_cols*nb_rows]) and an array of pointer rows (which size is [nb_rows])
  the \e ith element in the row array. row[i] is a pointer toward the \e ith
  "line" of the image (i.e., bitmap +i*nb_cols )

  \image html image-data-structure.gif
  \image latex image-data-structure.ps  "The image structure" width=10cm

  \if 0
  bitmap --------------------|
                             v
               -----         --------------------------------------------
  row -------> | x-|-------->| | | | | | | | |  ...     | | | | | | | | |
               -----         --------------------------------------------
	       | x-|-------->| | | | | | | | |  ...     | | | | | | | | |
               -----         --------------------------------------------
	       |   |
               -----
	       |   |
               -----
	       |   |
               -----
                 :
  \endif

  Such a structure allows a fast access to each element of the image.
  If \e i is the ith rows and \e j the jth column, the value of this pixel
  is given by I[i][j] (that is equivalent to row[i][j]).

  This is a template class, therefore the type of each  element of the
  array is not a priori defined.

  The CMotion2DImage class is only provided for convenience in order to build
  the Motion2D example programs. Therefore, image input/output manipulations
  are restricted and only supported for PNG (see ReadPNG() and WritePNG()
  functions), RAW and PNM file format.

  A RAW image file contains simply the list of pixel values with no
  header. That is why the number of columns and of rows must be given by the
  user. For RAW file format images, two sets of functions exist, one for the
  traditional 8-bits images named RAW8 (see functions ReadRAW8() and
  WriteRAW8()), the other for 9 to 16 bits images named RAW16 (see functions
  ReadRAW16() and WriteRAW16()).

  The different PNM formats are PGM P5
  (see functions ReadPGM() and WritePGM()) and PPM P6 (see functions ReadPPM()
  or WritePPM()). To know more about this image file format: \e man \e png, \e
  man \e pgm or \e man \e ppm.


*/

#ifndef CMotion2DImage_h
#define CMotion2DImage_h

#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif
#include <math.h>

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

#ifndef FILENAME_MAX
#  define FILENAME_MAX 1024
#endif


#define DEBUG_LEVEL1 0

/*!
  Sort in an array.

  \param ra The array to sort.
  \param n Array size

  \return false if the array size is null, true if the sort was successfully
  achieved.

*/
template <class T> bool sort(unsigned n, T* ra)
{
  unsigned long l,j,ir,i;
  T rra;
  unsigned long min,max;

  if (DEBUG_LEVEL1) cout << "Begin sort()" << endl;
  if (n == 0)
    return false;

  if (n == 1)
    return true;

  min=n+1;
  max= 0; // -1 Modif FS 13/05/04
  l=(n >> 1)+1;
  ir=n;
  for (;;) {
    if (l > 1)
      rra=ra[--l];
    else {
      rra=ra[ir];
      ra[ir]=ra[1];
      if (ir>max) max=ir;
      if (ir<min) min=ir;
      if (--ir == 1) {
	ra[1]=rra;

	return true;
      }
    }
    i=l;
    j=l << 1;
    while (j <= ir) {
      if (j+1>max) max=j+1;
      if (j<min) min=j;
      if (j < ir && ra[j] < ra[j+1]) ++j;
      if (rra < ra[j]) {
	if (i>max) max=i;
	if (i<min) min=i;
	ra[i]=ra[j];
	j += (i=j);
      }
      else j=ir+1;
    }
    if (i>max) max=i;
    if (i<min) min=i;
    ra[i]=rra;
  }
  if (DEBUG_LEVEL1) cout << "End sort()" << endl;
  return true;
}




template<class Type>
class CMotion2DImage
{
private:
  unsigned npixels;
  unsigned ncols ;
  unsigned nrows ;

public:
  Type *bitmap ; /*!< The bitmap array associated to the image. */
  Type **row ; /*!< The row array to access to the first element of a line. */

public:
  CMotion2DImage() ;
  //copy constructors
  CMotion2DImage(const CMotion2DImage<Type>&);
  CMotion2DImage(unsigned nb_rows, unsigned nb_cols);
  CMotion2DImage(unsigned nb_rows, unsigned nb_cols, Type value);
  ~CMotion2DImage() ;

  bool Init(unsigned nb_rows, unsigned nb_cols);
  bool Init(unsigned nb_rows, unsigned nb_cols, Type value);

  bool Resize(unsigned nb_rows, unsigned nb_cols);
  void Subsample();
  bool And(CMotion2DImage<Type> &A, CMotion2DImage<Type> &B);
  bool And(CMotion2DImage<Type> &A, CMotion2DImage<Type> &B, Type label);
  bool MedianFilter(unsigned filterRowSize, unsigned filterColSize);

  /*!
    Returns the number of rows of an image.

    \sa GetCols()
  */
  inline unsigned GetRows() const { return nrows ; }
  /*!
    Returns the number of columns of an image.

    \sa GetRows()
  */
  inline unsigned GetCols() const { return ncols ; }
  /*!

    Returns the number of pixels in the image. This number reflects the image
    size.

  */
  inline unsigned GetNumberOfPixel() const{ return npixels; }

  /*!

    Allows a direct access to the bitmap to change the value of the pixel
    located at the \e ith line and \e jth column: [i][j] = (row[i])[j].

  */
  Type *operator[](unsigned i) {return row[i];}
  /*!

    Allows a direct access to the bitmap to get the value of the pixel
    located at the \e ith line and \e jth column: [i][j] = (row[i])[j].

  */
  const  Type *operator[](unsigned i) const {return row[i];}


  void operator=(Type x);

  void operator=(const CMotion2DImage<Type> &image);

  CMotion2DImage<Type> operator-(const CMotion2DImage<Type> &image);

private:
  /*!
    Set the number of columns of an image.

    \sa SetRows(), CMotion2DImage(unsigned, unsigned)
  */
  void SetCols(unsigned nb_cols) { ncols = nb_cols ; } ;
  /*!
    Set the number of rows of an image.

    \sa SetCols(), CMotion2DImage(unsigned, unsigned)
  */
  void SetRows(unsigned nb_rows) { nrows = nb_rows ; } ;


} ;


#include "CMotion2DImage_base.cpp"

// Include for image ios
#include "Motion2DImage_PNM.h"
#include "Motion2DImage_RAW.h"
#include "Motion2DImage_PNG.h"

#undef DEBUG_LEVEL1


#endif
