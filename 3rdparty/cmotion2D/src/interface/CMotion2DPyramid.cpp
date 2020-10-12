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
  \file CMotion2DPyramid.cpp
  \brief Definition of the CMotion2DPyramid class.
*/

/*!

  \class CMotion2DPyramid

  \brief The CMotion2DPyramid class provides image pyramids.

  The CMotion2DPyramid class provides the Gaussian pyramid \f$I_g\f$ and
  spatial gradient pyramids \f$\nabla I_g^t = ({I_g}_x, {I_g}_y) =
  (\frac{\partial I_g}{\partial x}, \frac{\partial I_g}{\partial y}) \f$ for an
  image \f$I\f$.

*/

#ifdef __SunOS_
# include <iostream.h>
# include <fstream.h>
#else
# include <iostream>
# include <fstream>
#endif

#include <stdlib.h>

#include "type.h"
#include "constant.h"
#include "macro.h"


#include "../pyramide/pyramide.h"
#include "../pyramide/filt_gauss.h"
#include "../memoire/memoire.h"

#include <cmotion2d/CMotion2DPyramid.h>

/*!

  Constructor.

*/
CMotion2DPyramid::CMotion2DPyramid()
  : allocated(false)
{
  nrows = 0;
  ncols = 0;
  nlevels = 0;
}

/*!

  Copy Constructor.

*/
CMotion2DPyramid::CMotion2DPyramid(const CMotion2DPyramid & pyramid)
{
  allocated = false;

  allocate(pyramid.nrows, pyramid.ncols, pyramid.nlevels);

  for (int i=0; i <= (pyramid.nlevels-1); i ++) {
    pyr_ima[i].nbco = pyramid.pyr_ima[i].nbco;
    pyr_ima[i].nbli = pyramid.pyr_ima[i].nbli;
    pyr_gx[i].nbco  = pyramid.pyr_gx[i].nbco;
    pyr_gx[i].nbli  = pyramid.pyr_gx[i].nbli;
    pyr_gy[i].nbco  = pyramid.pyr_gy[i].nbco;
    pyr_gy[i].nbli  = pyramid.pyr_gy[i].nbli;

    int n_elements = pyr_ima[i].nbco * pyr_ima[i].nbli;

    memcpy(pyr_ima[i].ad, pyramid.pyr_ima[i].ad, n_elements * sizeof(short));
    memcpy(pyr_gx[i].ad,  pyramid.pyr_gx[i].ad,  n_elements * sizeof(float));
    memcpy(pyr_gy[i].ad,  pyramid.pyr_gy[i].ad,  n_elements * sizeof(float));
  }
}

/*!

  Destructor.

*/
CMotion2DPyramid::~CMotion2DPyramid()
{
  destroy();
}

/*!

  Allocate memory to construct a low-pass Gaussian pyramid \f$I_g\f$ and
  spatial gradient pyramids \f$\nabla I_g\f$ of an image \f$I\f$. The number
  of levels \f$L\f$ to built in a pyramid is given by \e nlevels. The coarsest
  pyramid level is \f$L-1\f$. The finest level is 0. At this level, images has
  a size of [\e nrows, \e ncols]. From level \f$l\f$ to \f$l+1\f$ the number of
  rows and columns are divided by 2.

  To construct the pyramids you have to use build(const unsigned char *)

  Return an error if the memory allocation can not be done.

  \sa build(const unsigned char *)

*/
TPyramidError CMotion2DPyramid::allocate(int nrows, int ncols, int nlevels)
{
  if (nlevels >= NLEVELS_MAX) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DPyramid::allocate() \n\tCan not construct more \
than %d levels in spatio-temporal pyramids...", NLEVELS_MAX);
    __panic(message);
    return PYR_TOO_MUCH_LEVELS;
  }

  if (allocated) {
    __panic((char*)"CMotion2DPyramid::allocte()\n\tMemory pyramid is \
allocated yet. Destroy before...");
    return PYR_ALLOCATION_DONE;
  }

  this->nlevels = nlevels;
  this->ncols   = ncols;
  this->nrows   = nrows;

  /* les champs nbco, nbli seront reremplis par Mem_pyramide() */
  pyr_ima[0].nbco = 0;
  pyr_ima[0].nbli = 0;
  pyr_ima[0].ad   = 0;

  int level_max = nlevels - 1;
  /* allocation m�moire pour les pyramides image et de gradients */
  if (Mem_pyramide(pyr_ima, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gx, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gy, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;

  // Update the number of levels
  this->nlevels = level_max + 1;

  allocated = true;

  return PYR_NO_ERROR;
}

/*!

  Build a low-pass Gaussian image pyramid \f$I_g\f$ and spatial image gradient
  pyramids \f$\nabla I_g\f$ based on \e image \f$I\f$.

  \warning The allocation is assumed to be done by allocate(). The size of the
  \e image must be conform to the number of rows and columns passed to
  allocate().

  Return an error if the pyramid construction can not be done.

  \sa allocate()

*/
TPyramidError CMotion2DPyramid::build(const unsigned char *image)
{
  int i;
  double var_gauss = 1.0;  /* Variance pour le lissage de l'image de base.*/

  if (allocated) {
    for (i=0; i < (pyr_ima[0].nbli * pyr_ima[0].nbco); i++)
      pyr_ima[0].ad[i] = (short) image[i];

    /* Etablissement de la pyramide de l'image 1 */
    // Warning: nlevels can be modified by Etablit_pyr()
    if (Etablit_pyr(pyr_ima, pyr_gx, pyr_gy, nlevels,
		    var_gauss, 0.4, 3, 1, 0) == false)
      return PYR_NOT_BUILD;
  }
  else {
    __panic((char*)"CMotion2DPyramid::Build() \n\tMemory pyramid is not \
allocated. Allocate before...");
    return PYR_NOT_ALLOCATED;
  }

  return PYR_NO_ERROR;
}

/*!

  Allocate the memory by calling allocate() and build a low-pass Gaussian image
  pyramid \f$I_g\f$ and spatial image gradient pyramids \f$\nabla I_g\f$
  based on \e image \f$I\f$ which dimension is given by \e nrows and \e
  ncols. The number of levels to construct is given by \e nlevels.

  Return an error if the pyramid construction can not be done.

  \sa allocate()

*/
TPyramidError CMotion2DPyramid::build(const unsigned char *image,
				      int nrows, int ncols, int nlevels)
{
  int i;
  double var_gauss = 1.0;  /* Variance pour le lissage de l'image de base.*/

  if( nlevels >= NLEVELS_MAX) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DPyramid::Build() \n\tCan not construct more \
than %d levels in spatio-temporal pyramids...", NLEVELS_MAX);
    __panic(message);
    return PYR_TOO_MUCH_LEVELS;
  }

  if (allocated) {
    __panic((char*)"CMotion2DPyramid::Build() \n\tMemory pyramid is \
allocated yet. Destroy before...");
    return(PYR_ALLOCATION_DONE);
  }

  this->nlevels = nlevels;
  this->ncols   = ncols;
  this->nrows   = nrows;

  /* les champs nbco, nbli seront reremplis par Mem_pyramide() */
  pyr_ima[0].nbli=0;
  pyr_ima[0].nbco=0;
  pyr_ima[0].ad=0;

  int level_max = nlevels - 1;
  /* allocation m�moire pour les pyramides image et de gradients */
  if (Mem_pyramide(pyr_ima, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gx, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gy, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;

  // Update the number of levels
  this->nlevels = level_max + 1;

  allocated = true;

  for (i=0; i < (nrows * ncols); i++)
    pyr_ima[0].ad[i] = (short) image[i];

  /* Etablissement de la pyramide de l'image 1 */
  if (Etablit_pyr(pyr_ima, pyr_gx, pyr_gy, this->nlevels,
		  var_gauss, 0.4, 3, 1, 0) == false)
    return PYR_NOT_BUILD;

  return PYR_NO_ERROR;
}

/*!

  Build a low-pass Gaussian image pyramid \f$I_g\f$ and spatial image gradient
  pyramids \f$\nabla I_g\f$ based on \e image \f$I\f$.

  \warning The allocation is assumed to be done by allocate(). The size of the
  \e image must be conform to the number of rows and columns passed to
  allocate().

  Return an error if the pyramid construction can not be done.

  \sa allocate()

*/
TPyramidError CMotion2DPyramid::build(const short *image)
{
  int i;
  double var_gauss = 1.0;  /* Variance pour le lissage de l'image de base.*/

  if (allocated) {
    for (i=0; i < (pyr_ima[0].nbli * pyr_ima[0].nbco); i++)
      pyr_ima[0].ad[i] = image[i];

    /* Etablissement de la pyramide de l'image 1 */
    // Warning: nlevels can be modified by Etablit_pyr()
    if (Etablit_pyr(pyr_ima, pyr_gx, pyr_gy, nlevels,
		    var_gauss, 0.4, 3, 1, 0) == false)
      return PYR_NOT_BUILD;
  }
  else {
    __panic((char*)"CMotion2DPyramid::Build() \n\tMemory pyramid is not \
allocated. Allocate before...");
    return PYR_NOT_ALLOCATED;
  }

  return PYR_NO_ERROR;
}

/*!

  Allocate the memory by calling allocate() and build a low-pass Gaussian image
  pyramid \f$I_g\f$ and spatial image gradient pyramids \f$\nabla I_g\f$
  based on \e image \f$I\f$ which dimension is given by \e nrows and \e
  ncols. The number of levels to construct is given by \e nlevels.

  Return an error if the pyramid construction can not be done.

  \sa allocate()

*/
TPyramidError CMotion2DPyramid::build(const short *image,
				      int nrows, int ncols, int nlevels)
{
  int i;
  double var_gauss = 1.0;  /* Variance pour le lissage de l'image de base.*/

  if( nlevels >= NLEVELS_MAX) {
    char message[FILENAME_MAX];
    sprintf(message, "CMotion2DPyramid::Build() \n\tCan not construct more \
than %d levels in spatio-temporal pyramids...", NLEVELS_MAX);
    __panic(message);
    return PYR_TOO_MUCH_LEVELS;
  }

  if (allocated) {
    __panic((char*)"CMotion2DPyramid::Build() \n\tMemory pyramid is \
allocated yet. Destroy before...");
    return(PYR_ALLOCATION_DONE);
  }

  this->nlevels = nlevels;
  this->ncols   = ncols;
  this->nrows   = nrows;

  /* les champs nbco, nbli seront reremplis par Mem_pyramide() */
  pyr_ima[0].nbli=0;
  pyr_ima[0].nbco=0;
  pyr_ima[0].ad=0;

  int level_max = nlevels - 1;
  /* allocation m�moire pour les pyramides image et de gradients */
  if (Mem_pyramide(pyr_ima, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gx, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;
  if (Mem_pyramide_float(pyr_gy, nrows, ncols, level_max) == false)
    return PYR_NOT_ENOUGH_MEMORY;

  // Update the number of levels
  this->nlevels = level_max + 1;

  allocated = true;

  for (i=0; i < (nrows * ncols); i++)
    pyr_ima[0].ad[i] = image[i];

  /* Etablissement de la pyramide de l'image 1 */
  if (Etablit_pyr(pyr_ima, pyr_gx, pyr_gy, this->nlevels,
		  var_gauss, 0.4, 3, 1, 0) == false)
    return PYR_NOT_BUILD;

  return PYR_NO_ERROR;
}
/*!

  Delete the space memory associate to the image pyramids.

*/
void CMotion2DPyramid::destroy()
{
  if (allocated) {
    free_p   (pyr_ima, nlevels);
    free_p_fl(pyr_gx,  nlevels);
    free_p_fl(pyr_gy,  nlevels);

    ncols   = 0;
    nrows   = 0;
    nlevels = 0;

    allocated = false;
  }
}

/*!

  Return the number of levels \f$L\f$ in a pyramid.

*/
int CMotion2DPyramid::getNumberOfLevels() const
{
  return nlevels;
}

/*!

  Get the number of rows for the specified \e level in the pyramid. The
  floor level is referenced by the number zero.

  Different error messages could be returned.

  \sa getNumberOfCols()

*/
TPyramidError CMotion2DPyramid::getNumberOfRows(int & nrows, int level) const
{
  nrows = 0;
  if (allocated) {
    if (level < nlevels) {
      nrows = pyr_ima[level].nbli;
      return PYR_NO_ERROR;
    }
    else
      return PYR_LEVEL_NOT_ACCESSIBLE;
  }
  else
    return PYR_NOT_ALLOCATED;
}

/*!

  Get the number of columns for the specified \e level in the pyramid. The
  floor level is referenced by the number zero.

  Different \e error messages could be returned.

  \sa getNumberOfRows()

*/
TPyramidError CMotion2DPyramid::getNumberOfCols(int & ncols, int level) const
{
  ncols = 0;
  if (allocated) {
    if (level < nlevels) {
      ncols = pyr_ima[level].nbco;
      return PYR_NO_ERROR;
    }
    else
      return PYR_LEVEL_NOT_ACCESSIBLE;
  }
  else
    return PYR_NOT_ALLOCATED;
}

/*!

  Return the address of the table containing the spatial gradient under the row
  axes for the specified pyramid level \f${I_g}_y(level)\f$. The size of the
  table containing these data is given by the product getNumberOfRows(level) *
  getNumberOfCols(level).

  Different \e error messages could be returned.

  \sa getNumberOfRows(), getNumberOfCols(),
  getColsSpatialGradientDataAddress(), getGaussianImageDataAddress()

*/
float *CMotion2DPyramid::
getRowsSpatialGradientDataAddress(TPyramidError & error, int level)
{
  float *data = NULL;
  if (allocated) {
    if (level < nlevels) {
      data = pyr_gy[level].ad;
    }
    else
      error = PYR_LEVEL_NOT_ACCESSIBLE;
  }
  else
    error = PYR_NOT_ALLOCATED;

  error = PYR_NO_ERROR;
  return data;
}

/*!

  Return the address of the table containing the spatial gradient under the
  column axes for the specified pyramid level \f${I_g}_x(level)\f$. The size of
  the table containing these data is given by the product
  getNumberOfRows(level) * getNumberOfCols(level).

  Different \e error messages could be returned.

  \sa getNumberOfRows(), getNumberOfCols(),
  getRowsSpatialGradientDataAddress(), getGaussianImageDataAddress()

*/
float *CMotion2DPyramid::
getColsSpatialGradientDataAddress(TPyramidError & error, int level)
{
  float *data = NULL;

  if (allocated) {
    if (level < nlevels)
      data = pyr_gx[level].ad;
    else
      error = PYR_LEVEL_NOT_ACCESSIBLE;
  }
  else
    error = PYR_NOT_ALLOCATED;

  error = PYR_NO_ERROR;
  return data;
}

/*!

  Return the address of the table containing the low-pass Gaussian image for
  the specified pyramid level \f$I_g(level)\f$. The size of the table
  containing these data is given by the product getNumberOfRows(level) *
  getNumberOfCols(level).

  Different \e error messages could be returned.

  \sa getNumberOfRows(), getNumberOfCols(),
  getColsSpatialGradientDataAddress(), getRowsSpatialGradientDataAddress()

*/
short *CMotion2DPyramid::
getGaussianImageDataAddress(TPyramidError & error, int level)
{
  short *data = NULL;
  if (allocated) {
    if (level < nlevels)
      data = pyr_ima[level].ad;
    else
      error = PYR_LEVEL_NOT_ACCESSIBLE;
  }
  else
    error = PYR_NOT_ALLOCATED;

  error = PYR_NO_ERROR;
  return data;
}

/*!

  Reaffect the pointer of the pyramids in order that the content of the
  pyramids are exchanged.

  \sa destroy(), allocate(), build()

*/
CMotion2DPyramid & CMotion2DPyramid::exchange(CMotion2DPyramid & pyramid)
{
  TImageShort tmp_pyr_ima[NLEVELS_MAX]; // Temporal gradiens
  TImageFloat tmp_pyr_gx [NLEVELS_MAX]; // Spatial gradiens in x (cols)
  TImageFloat tmp_pyr_gy [NLEVELS_MAX]; // Spatial gradiens in y (rows)

  int tmp_nlevels; // Number of pyramid levels.
  int tmp_nrows;   // Number of rows for the floor level.
  int tmp_ncols;   // Number of cols for the floor level.

  bool tmp_allocated;

  // Save the current pyramid characteristics
  tmp_nlevels = nlevels;
  tmp_nrows   = nrows;
  tmp_ncols   = ncols;

  tmp_allocated = allocated;

  if (tmp_allocated) {
    for (int i=0; i < nlevels; i++) {
      tmp_pyr_ima[i].nbli = pyr_ima[i].nbli;
      tmp_pyr_ima[i].nbco = pyr_ima[i].nbco;
      tmp_pyr_gx[i].nbli  = pyr_gx[i].nbli;
      tmp_pyr_gx[i].nbco  = pyr_gx[i].nbco;
      tmp_pyr_gy[i].nbli  = pyr_gy[i].nbli;
      tmp_pyr_gy[i].nbco  = pyr_gy[i].nbco;

      tmp_pyr_ima[i].ad = pyr_ima[i].ad;
      tmp_pyr_gx[i].ad  = pyr_gx[i].ad;
      tmp_pyr_gy[i].ad  = pyr_gy[i].ad;
    }
  }

  // Current pyramid (this) recieve the pyramid characteristics
  nlevels = pyramid.nlevels;
  nrows   = pyramid.nrows;
  ncols   = pyramid.ncols;

  allocated = pyramid.allocated;

  if (allocated) {
    for (int i=0; i < pyramid.nlevels; i++) {
      pyr_ima[i].nbli = pyramid.pyr_ima[i].nbli;
      pyr_ima[i].nbco = pyramid.pyr_ima[i].nbco;
      pyr_gx[i].nbli  = pyramid.pyr_gx[i].nbli;
      pyr_gx[i].nbco  = pyramid.pyr_gx[i].nbco;
      pyr_gy[i].nbli  = pyramid.pyr_gy[i].nbli;
      pyr_gy[i].nbco  = pyramid.pyr_gy[i].nbco;

      pyr_ima[i].ad = pyramid.pyr_ima[i].ad;
      pyr_gx[i].ad  = pyramid.pyr_gx[i].ad;
      pyr_gy[i].ad  = pyramid.pyr_gy[i].ad;
    }
  }

  // the pyramid characteristics receive this
  pyramid.nlevels = tmp_nlevels;
  pyramid.nrows   = tmp_nrows;
  pyramid.ncols   = tmp_ncols;

  pyramid.allocated = tmp_allocated;

  if (tmp_allocated) {
    for (int i=0; i < tmp_nlevels; i++) {
      pyramid.pyr_ima[i].nbli = tmp_pyr_ima[i].nbli;
      pyramid.pyr_ima[i].nbco = tmp_pyr_ima[i].nbco;
      pyramid.pyr_gx[i].nbli  = tmp_pyr_gx[i].nbli;
      pyramid.pyr_gx[i].nbco  = tmp_pyr_gx[i].nbco;
      pyramid.pyr_gy[i].nbli  = tmp_pyr_gy[i].nbli;
      pyramid.pyr_gy[i].nbco  = tmp_pyr_gy[i].nbco;

      pyramid.pyr_ima[i].ad = tmp_pyr_ima[i].ad;
      pyramid.pyr_gx[i].ad  = tmp_pyr_gx[i].ad;
      pyramid.pyr_gy[i].ad  = tmp_pyr_gy[i].ad;
    }
  }

  return *this;
}

/*!


  Print an error message to stderr and return false.

*/
void CMotion2DPyramid::__panic(char *message)
{
  cerr << message << endl;
}

