/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <cmotion2d/CMotion2DModel.h>
#include <cmotion2d/FieldVector.h>

/*!
  \file FieldVector.cpp
  \brief Definition of field vectors input/output functions.
*/
/*!

  Write the corresponding motion model optic flow field in a file which name is
  specified by \e filename. The optic flow field is only computed on the
  estimation support S. The estimation support corresponds to the pixels of S
  whose values are equal to \e label.

  \param model Estimated 2D parametric motion model.

  \param S refers to the estimation support \f$R_t\f$. In other terms, S
  indicates which pixels of the image \f$I_t\f$ are taken into account in the
  estimation process. It usually consists of the whole image. In that case, S
  must be set to \e label. But, if required, it can also be restricted to a
  specific area of the image. In that case, all the pixels belonging to this
  area must be set to \e label. The other pixels must be set to another value
  than \e label.

  \param label Value of area where the motion model was estimated.

  \param filename Name of the file containing the optic flow field.

  \return false if an error occurs, or true otherwise.
*/
bool WriteFieldVector(CMotion2DModel &model,
		      CMotion2DImage<unsigned char> &S, unsigned char label,
		      const char *filename)
{
  short		CL, NL, cl, nl;
  FILE          *fd;
  char 		entete[4];
  char 		*buffer, c, *pt;
  unsigned char c1;
  double	*field_u, *field_v;
  double	*pf;
  int           ierr;
  double        d_row,d_col;  // Displacement
  unsigned      nrows = S.GetRows();
  unsigned      ncols = S.GetCols();
  unsigned      size = nrows*ncols;
  unsigned i;

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  field_u = (double *) calloc((unsigned int)(size), sizeof(double));
  field_v = (double *) calloc((unsigned int)(size), sizeof(double));

  for (unsigned col=0; col < ncols; col++) {
    for (unsigned row=0; row < nrows; row++) {
      if (S[row][col] == label) {
	model.getDisplacement((double)row, (double)col, d_row, d_col);
	*(field_u + row * ncols + col) = d_col;
	*(field_v + row * ncols + col) = d_row;
      }
    }
  }

  if ((buffer = (char *) calloc((unsigned int)(2*ncols*nrows),sizeof(char))) == NULL) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't allocate memory\n");
    fclose(fd);
    free(field_u);
    free(field_v);
    return false;
  }

  CL = (short )(ncols / 256);
  cl =  (short)(ncols - 256*CL);
  NL = (short)(nrows / 256);
  nl =  (short)(nrows - 256*NL);

  entete[0] = (char)(NL);
  entete[1] = (char)(nl);
  entete[2] = (char)(CL);
  entete[3] = (char)(cl);

  unsigned nbyte = 4;
  ierr = fwrite(entete, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  pt = buffer; pf = field_u;
  for (i=0 ; i<nrows ; i++)  for (unsigned j=0 ; j<ncols ; j++) {
    (*pf) += (*pf >= 0.0) ? 1 : -1;
    c = (char) *pf;
    if ( *pf >= 0) c1 = (char)((*pf -  c) * 200.0);
    else c1 = (char)((c - *pf) * 200.0);
    *pt++ = c;
    *pt++ = c1;
    pf ++;
  }

  nbyte = 2*ncols*nrows;
  ierr = fwrite(buffer, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WritePGM: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  pt = buffer; pf = field_v;
  for (i=0 ; i<nrows ; i++)  for (unsigned j=0 ; j<ncols ; j++) {
    (*pf) += (*pf >= 0.0) ? 1 : -1;
    c = (char) *pf;
    if ( *pf >= 0)  c1 = (char)((*pf -  c) * 200.0);
    else c1 = (char)((c - *pf) * 200.0);
    *pt++ = c;
    *pt++ = c1;
    pf ++;
  }

  ierr = fwrite(buffer, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WritePGM: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  free(field_u);
  free(field_v);
  free(buffer);

  fclose(fd);

  return true;
}

/*!

  Write the corresponding motion model optic flow field in a file which name is
  specified by \e filename. The optic flow field is only computed on the
  estimation support S. The estimation support corresponds to the pixels of S
  whose values are equal to \e label.

  \param model Estimated 2D parametric motion model.

  \param S refers to the estimation support \f$R_t\f$. In other terms, S
  indicates which pixels of the image \f$I_t\f$ are taken into account in the
  estimation process. It usually consists of the whole image. In that case, S
  must be set to \e label. But, if required, it can also be restricted to a
  specific area of the image. In that case, all the pixels belonging to this
  area must be set to \e label. The other pixels must be set to another value
  than \e label.

  \param label Value of area where the motion model was estimated.

  \param filename Name of the file containing the optic flow field.

  \return false if an error occurs, or true otherwise.
*/
bool WriteFieldVector(CMotion2DModel &model,
		      CMotion2DImage<short> &S, short label,
		      const char *filename)
{
  short		CL, NL, cl, nl;
  FILE          *fd;
  char 		entete[4];
  char 		*buffer, c, *pt;
  unsigned char c1;
  double	*field_u, *field_v;
  double	*pf;
  int           ierr;
  double        d_row,d_col;  // Displacement
  unsigned      nrows = S.GetRows();
  unsigned      ncols = S.GetCols();
  unsigned      size = nrows*ncols;
  unsigned i;

  fd = fopen(filename, "wb");

  if (fd == NULL) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't write to file %s\n",
	    filename);
    return false ;
  }

  field_u = (double *) calloc((unsigned int)(size), sizeof(double));
  field_v = (double *) calloc((unsigned int)(size), sizeof(double));

  for (unsigned col=0; col < ncols; col++) {
    for (unsigned row=0; row < nrows; row++) {
      if (S[row][col] == label) {
	model.getDisplacement((double)row, (double)col, d_row, d_col);
	*(field_u + row * ncols + col) = d_col;
	*(field_v + row * ncols + col) = d_row;
      }
    }
  }

  if ((buffer = (char *) calloc((unsigned int)(2*ncols*nrows),sizeof(char))) == NULL) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't allocate memory\n");
    fclose(fd);
    free(field_u);
    free(field_v);
    return false;
  }

  CL = (short )(ncols / 256);
  cl =  (short)(ncols - 256*CL);
  NL = (short)(nrows / 256);
  nl =  (short)(nrows - 256*NL);

  entete[0] = (char)(NL);
  entete[1] = (char)(nl);
  entete[2] = (char)(CL);
  entete[3] = (char)(cl);

  unsigned nbyte = 4;
  ierr = fwrite(entete, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WriteFieldVector: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  pt = buffer; pf = field_u;
  for (i=0 ; i<nrows ; i++)  for (unsigned j=0 ; j<ncols ; j++) {
    (*pf) += (*pf >= 0.0) ? 1 : -1;
    c = (char) *pf;
    if ( *pf >= 0) c1 = (char)((*pf -  c) * 200.0);
    else c1 = (char)((c - *pf) * 200.0);
    *pt++ = c;
    *pt++ = c1;
    pf ++;
  }

  nbyte = 2*ncols*nrows;
  ierr = fwrite(buffer, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WritePGM: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  pt = buffer; pf = field_v;
  for (i=0 ; i<nrows ; i++)  for (unsigned j=0 ; j<ncols ; j++) {
    (*pf) += (*pf >= 0.0) ? 1 : -1;
    c = (char) *pf;
    if ( *pf >= 0)  c1 = (char)((*pf -  c) * 200.0);
    else c1 = (char)((c - *pf) * 200.0);
    *pt++ = c;
    *pt++ = c1;
    pf ++;
  }

  ierr = fwrite(buffer, sizeof(char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fprintf(stderr, "Error in WritePGM: couldn't write %d bytes to file %s\n",
	    nbyte, filename);
    fclose(fd);
    free(field_u);
    free(field_v);
    free(buffer);
    return false;
  }

  free(field_u);
  free(field_v);
  free(buffer);

  fclose(fd);

  return true;
}

