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
  \file CMotion2DImage_base.cpp
  \brief Definition of the CMotion2DImage template class.
*/

/*!

  Default constructor. No memory allocation is done.

  \sa CMotion2DImage(unsigned, unsigned), CMotion2DImage(unsigned, unsigned, Type),

*/

#include <string.h>

template<class Type>
CMotion2DImage<Type>::CMotion2DImage()
{
  bitmap = NULL ;
  row = NULL ;
  nrows = ncols = 0 ;
  npixels = 0;
}

/*!

  Copy constructor.

*/
template<class Type>
CMotion2DImage<Type>::CMotion2DImage(const CMotion2DImage<Type>& I)
{
  bitmap = NULL ;
  row = NULL ;
  nrows = ncols = 0 ;
  npixels = 0;

  Resize(I.GetRows(),I.GetCols());
  unsigned i;
  memcpy(bitmap, I.bitmap, I.npixels*sizeof(Type)) ;
  for (i =0  ; i < nrows ; i++) row[i] = bitmap + i*ncols ;
}

/*!

  Constructor.

  Allocates memory for an image of \e nb_rows rows and \e nb_cols columns and
  set all the elements of the bitmap to 0.

  If the image was already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa CMotion2DImage(), Init(unsigned, unsigned)

*/
template<class Type>
CMotion2DImage<Type>::CMotion2DImage(unsigned nb_rows, unsigned nb_cols)
{
  bitmap = NULL ;
  row = NULL ;
  nrows = ncols = 0 ;
  npixels = 0;

  Init(nb_rows,nb_cols,0) ;
}

/*!

  Constructor.

  Allocates memory for an image of \e nb_rows rows and \e nb_cols columns and
  set all the elements of the bitmap to \e value.

  If the image was already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \sa CMotion2DImage(), Init(unsigned, unsigned)

*/
template<class Type>
CMotion2DImage<Type>::CMotion2DImage (unsigned nb_rows, unsigned nb_cols, Type value)
{
  bitmap = NULL ;
  row = NULL ;
  nrows = ncols = 0 ;
  Init(nb_rows,nb_cols,value) ;
}

/*!

  Destroys the image. Free the memory used by the image.

*/
template<class Type>
CMotion2DImage<Type>::~CMotion2DImage()
{
  if (row!=NULL) {
    delete [] row ;
    row = NULL;
  }
  if (bitmap!=NULL) {
    delete [] bitmap;
    bitmap = NULL;
  }
}




/*!

  Allocates memory for an image of \e nb_rows rows and \e nb_cols
  columns. Elements of the bitmap are not initialized.

  If the image was already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return true if successful, or false if memory cannot be allocated.

  \sa Init(unsigned, unsigned, Type)

*/
template<class Type>
bool
CMotion2DImage<Type>::Init(unsigned nb_rows, unsigned nb_cols)
{
  // Test image size differ
  if ((nb_rows != nrows) || (nb_cols != ncols)) {
    if (bitmap != NULL) {
      delete [] bitmap;
      bitmap = NULL;
    }
  }

  if (nb_rows != nrows) {
    if (row != NULL)  {
      delete [] row;
      row = NULL;
    }
  }

  SetCols(nb_cols) ;
  SetRows(nb_rows) ;

  npixels=ncols*nrows;

  if (bitmap == NULL) bitmap = new Type[npixels] ;
  if (bitmap == NULL)
  {
    cerr << "Error in CMotion2DImage::Init(): cannot allocate memory" << endl;
    return false ;
  }

  if (row == NULL) row = new  Type*[nrows] ;
  if (row == NULL)
  {
    cerr << "Error in CMotion2DImage::Init(): cannot allocate memory" << endl;
    return false;
  }

  unsigned i ;
  for ( i =0  ; i < nrows ; i++)
    row[i] = bitmap + i*ncols ;

  return true;
}

/*!

  Allocates memory for an image of \e nb_rows rows and \e nb_cols columns and
  set all elements of the bitmap to \e value.

  \return true if successful, or false if memory cannot be allocated.

  \sa Init(unsigned, unsigned)

*/
template<class Type>
bool
CMotion2DImage<Type>::Init(unsigned nb_rows, unsigned nb_cols, Type value)
{

  bool res = Init(nb_rows, nb_cols) ;
  if (res != true) return res ;

  for (unsigned i=0  ; i < npixels ;  i++)
    bitmap[i] = value ;

  return true ;
}

/*!

  Allocates memory for an image of \e nb_rows rows and \e nb_cols
  columns. Elements of the bitmap are not initialized.

  If the image was already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return true if successful, or false if memory cannot be allocated.

  \sa Init(unsigned, unsigned)

*/
template<class Type>
bool
CMotion2DImage<Type>::Resize(unsigned nb_rows, unsigned nb_cols)
{
  return Init(nb_rows, nb_cols);
}

/*!

  Computes the logical AND between the A et B images.

  C = A & B

  \return true if successful, or false if image size differs or if memory
  cannot be allocated.

*/
template<class Type>
bool
CMotion2DImage<Type>::And(CMotion2DImage<Type> &A, CMotion2DImage<Type> &B)
{
  if (A.GetRows() != B.GetRows())
    return false;
  if (A.GetCols() != B.GetCols())
    return false;

  bool res =  Init(A.GetRows(), A.GetCols());
  if (res != true)
    return res;

  for (unsigned i = 0; i < npixels; i ++) {
    bitmap[i] = A.bitmap[i] & B.bitmap[i];
  }

  return true;
}

/*!

  Computes a "specific" AND between the A et B images.

  \f[ C[i][j] = \left\{ \begin{array}{l} A[i][j] \mbox{\hspace{0.4cm}if\hspace{0.2cm}} B[i][j] = label \\ 0 \mbox{\hspace{1.2cm}} otherwise \end{array} \right. \f]


  \return true if successful, or false if image size differs or if memory
  cannot be allocated.

*/
template<class Type>
bool
CMotion2DImage<Type>::And(CMotion2DImage<Type> &A,
			  CMotion2DImage<Type> &B,
			  Type label)
{
  if (A.GetRows() != B.GetRows())
    return false;
  if (A.GetCols() != B.GetCols())
    return false;

  bool res =  Init(A.GetRows(), A.GetCols());
  if (res != true)
    return res;

  for (unsigned i = 0; i < npixels; i ++) {
    if (B.bitmap[i] == label)
      bitmap[i] = A.bitmap[i];
    else
      bitmap[i] = 0;
  }

  return true;
}

/*!

  Apply a median filter to the image. The filter size is given by the \e
  filterRowSize and \e filterColSize parameters.

  \return false if the filter size is bad (ie. filterRowSize or filterColSize
  are equal to zero), true if the filtering was successfully achieved.

*/
template<class Type>
bool CMotion2DImage<Type>::MedianFilter(unsigned filterRowSize,
					unsigned filterColSize)
{
  // Subsampled image size
  unsigned _nrows = GetRows();
  unsigned _ncols = GetCols();
  int ik, jl;
  Type *tab;

  if (filterRowSize < 1)
    return false;
  if (filterColSize < 1)
    return false;

  if ((filterRowSize * filterColSize) == 1)
    return true;

  int halfFilterRowSize = filterRowSize / 2;
  int halfFilterColSize = filterColSize / 2;

  CMotion2DImage<Type> I;
  I.Resize(_nrows, _ncols);

  tab = new Type[filterRowSize*filterColSize];
  for (unsigned i=0; i < _nrows; i++) {
    for (unsigned j=0; j < _ncols; j++) {

      unsigned co=0;
      for (int k = -halfFilterRowSize;
	   k < (int)(filterRowSize-halfFilterRowSize); k++) {
	for (int l = -halfFilterColSize;
	     l < (int)(filterColSize-halfFilterColSize); l++) {
	  ik=i+k;
	  jl=j+l;
	  if (ik >= 0 & ik < (int) _nrows & jl >= 0 &jl < (int) _ncols)
	    tab[co++] = (*this)[ik][jl];
	}
      }
      sort(co, tab-1);
      I[i][j] = tab[(co >> 1) + 1];
    }
  }

  (*this) = I;
  delete [] tab;
  return true;
}

/*!

  Assigns a copy of \e image to this image and returns a reference to this
  image.

*/
template<class Type>
void CMotion2DImage<Type>::operator=(const CMotion2DImage<Type> &image)
{
  Resize(image.GetRows(),image.GetCols()) ;

  unsigned i ;

  memcpy(bitmap, image.bitmap, image.npixels*sizeof(Type)) ;

  for (i=0; i<nrows; i++)
    row[i] = bitmap + i*ncols ;
}

/*!

  Set each element of the bitmap to \e x.
*/
template<class Type>
void CMotion2DImage<Type>::operator=(Type x)
{
  for (unsigned i=0 ; i < npixels ; i++)
    bitmap[i] = x ;
}
/*!

  Makes a difference between 2 images returns a reference to the difference
  image.

*/
template<class Type>
CMotion2DImage<Type> CMotion2DImage<Type>::operator-(const CMotion2DImage<Type> &image)
{

  CMotion2DImage<Type> D(GetRows(),GetCols());

  for (unsigned i=0; i<npixels; i++) {
    D.bitmap[i] = bitmap[i] - image.bitmap[i];
  }
  return D;
}
/*!

  Subsample the image. The subsampled image is given in I. The number of rows
  and columns of this image is divided by two. To determine the pixel value, we
  consider the mean value of the four neighbours.

*/
template<class Type>
void CMotion2DImage<Type>::Subsample()
{
  // Subsampled image size
  unsigned _nrows = GetRows() >> 1;
  unsigned _ncols = GetCols() >> 1;

  CMotion2DImage<Type> I;
  I.Resize(_nrows, _ncols);

  for (unsigned i=0; i < _nrows; i++) {
    for (unsigned j=0; j < _ncols; j++) {
      int u=0;
      for (unsigned k= 0; k <= 1;k++)
	for (unsigned l=0; l<= 1;l++) {
	  u += (*this)[k+(i<<1)][l+(j<<1)];
	}
      u = (u / 4);
      I[i][j] = u;
    }
  }

  (*this) = I;
}
